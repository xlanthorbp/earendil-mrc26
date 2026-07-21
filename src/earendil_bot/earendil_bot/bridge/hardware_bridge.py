#!/usr/bin/env python3
"""
Hardware Bridge Node for Earendil Bot (MRC 2026)
Raspberry Pi 5 -> Arduino Mega (engine.ino)
-----------------------------------------
1. /cmd_vel (Twist) mesajlarını engine.ino string komutlarına çevirir:
   - ileri_hizli / ileri_yavas
   - geri_hizli / geri_yavas
   - sag_hizli / sag_yavas
   - sol_hizli / sol_yavas
   - dur
2. Arduino seri portundan gelen sensör verilerini (Manyetometre / Heading) okur.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('hardware_bridge')

        # Parametreler (hardware_params.yaml'den yüklenir)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('heading_offset', 0.0)
        self.declare_parameter('motor_watchdog_timeout', 1.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.heading_offset = self.get_parameter('heading_offset').value
        self.motor_watchdog_timeout = self.get_parameter('motor_watchdog_timeout').value

        # Durum Değişkenleri
        self.last_cmd = "dur"
        self.last_cmd_time = time.time()
        self.serial_buffer = ""
        self.buffer_lock = threading.Lock()
        self.serial_lock = threading.Lock()

        # Seri Bağlantı
        self.ser = None
        self._connect_serial()

        # ROS 2 Yayıncı & Aboneler
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)
        self.led_sub = self.create_subscription(String, '/mode/led', self._led_callback, 10)
        self.mag_pub = self.create_publisher(Float32, '/mag/heading', 10)

        # Seri Port Okuma Thread'i
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()

        # Motor Zaman Aşımı (Watchdog) Kontrolü
        self.create_timer(0.2, self._keepalive)

        self.get_logger().info(f'Hardware Bridge Düğümü Başlatıldı. Arduino Port: {self.port}')

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Arduino'ya bağlandı: {self.port} ({self.baudrate} baud)")
        except serial.SerialException as e:
            self.get_logger().error(f"Arduino seri port bağlantı hatası: {e}")
            self.ser = None

    # ==================================================
    # MOTOR KOMUTLARI (ROS 2 Twist -> Arduino engine.ino)
    # ==================================================
    def _cmd_callback(self, msg: Twist):
        if not self.ser:
            return

        v = msg.linear.x
        w = msg.angular.z

        cmd = "dur"

        # Dönüş komutu (Açısal hız z)
        if abs(w) > 0.05:
            suffix = "hizli" if abs(w) > 0.4 else "yavas"
            cmd = f"sol_{suffix}" if w > 0 else f"sag_{suffix}"

        # İleri / Geri komutu (Çizgisel hız x)
        elif abs(v) > 0.05:
            suffix = "hizli" if abs(v) > 0.4 else "yavas"
            cmd = f"ileri_{suffix}" if v > 0 else f"geri_{suffix}"

        else:
            cmd = "dur"

        if cmd != self.last_cmd:
            self._send_raw(cmd)
            self.last_cmd = cmd
            self.last_cmd_time = time.time()
            self.get_logger().info(f"Arduino Komutu: {cmd}")
        else:
            self.last_cmd_time = time.time()

    def _led_callback(self, msg: String):
        if self.ser:
            mode_cmd = f"LED:{msg.data.upper()}"
            self._send_raw(mode_cmd)

    def _send_raw(self, cmd: str):
        try:
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    self.ser.write((cmd + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Seri port yazma hatası: {e}")

    def _keepalive(self):
        if self.last_cmd and self.last_cmd != "dur":
            if time.time() - self.last_cmd_time > self.motor_watchdog_timeout:
                self.last_cmd = "dur"
                self._send_raw("dur")
                self.get_logger().warn("Motor komutu zaman aşımı! Araç durduruldu.")

    # ==================================================
    # SERİ PORT OKUYUCU (Arduino -> Python)
    # ==================================================
    def _serial_reader(self):
        while rclpy.ok():
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                continue

            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    with self.serial_lock:
                        chunk = self.ser.read(waiting).decode('ascii', errors='ignore')

                    lines_to_process = []
                    with self.buffer_lock:
                        self.serial_buffer += chunk
                        while '\n' in self.serial_buffer:
                            line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                            lines_to_process.append(line.strip())

                    for line in lines_to_process:
                        self._parse_arduino_line(line)
                else:
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f"Seri okuma hatası: {e}")
                time.sleep(0.5)

    def _parse_arduino_line(self, line: str):
        if line.startswith("HEADING:") or line.startswith("MAG:"):
            try:
                val_str = line.split(":")[1]
                heading_deg = float(val_str) + self.heading_offset
                heading_deg = heading_deg % 360.0

                msg = Float32()
                msg.data = heading_deg
                self.mag_pub.publish(msg)
            except (IndexError, ValueError):
                pass

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self._send_raw("dur")
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
