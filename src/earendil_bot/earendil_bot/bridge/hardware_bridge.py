#!/usr/bin/env python3
"""
Hardware Bridge Node for Earendil Bot (MRC 2026)
Raspberry Pi 5 -> Arduino Mega (engine.ino) & Arduino Uno (magnetometer.ino)
---------------------------------------------------------------------------
1. /cmd_vel (Twist) mesajlarını Arduino Mega'ya (engine.ino) iletir:
   - ileri_hizli / ileri_yavas
   - geri_hizli / geri_yavas
   - sag_hizli / sag_yavas
   - sol_hizli / sol_yavas
   - dur
2. Arduino Uno'dan (magnetometer.ino) gelen sensör verilerini (Manyetometre / Heading) okur.
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
        self.declare_parameter('port', '/dev/ttyACM0')      # Arduino Mega (Motorlar)
        self.declare_parameter('mag_port', '/dev/ttyUSB0')  # Arduino Uno (Manyetometre)
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('heading_offset', 0.0)
        self.declare_parameter('motor_watchdog_timeout', 1.0)
        self.declare_parameter('min_pwm', 80)
        self.declare_parameter('max_pwm', 200)

        self.port = self.get_parameter('port').value
        self.mag_port = self.get_parameter('mag_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.heading_offset = self.get_parameter('heading_offset').value
        self.motor_watchdog_timeout = self.get_parameter('motor_watchdog_timeout').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value

        # Durum Değişkenleri
        self.last_cmd = "MOTOR:STOP"
        self.last_cmd_time = time.time()
        self.last_sent_time = 0.0
        self.serial_lock = threading.Lock()

        # Seri Bağlantılar
        self.ser_mega = None
        self.ser_uno = None
        self._connect_serials()

        # ROS 2 Yayıncı & Aboneler
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)
        self.led_sub = self.create_subscription(String, '/mode/led', self._led_callback, 10)
        self.mag_pub = self.create_publisher(Float32, '/mag/heading', 10)

        # Seri Port Okuma Thread'leri
        self.reader_threads = []
        if self.ser_uno and self.ser_uno != self.ser_mega:
            t_uno = threading.Thread(target=self._serial_reader_uno, daemon=True)
            t_uno.start()
            self.reader_threads.append(t_uno)

        if self.ser_mega:
            t_mega = threading.Thread(target=self._serial_reader_mega, daemon=True)
            t_mega.start()
            self.reader_threads.append(t_mega)

        # Motor Zaman Aşımı (Watchdog) Kontrolü
        self.create_timer(0.2, self._keepalive)

        self.get_logger().info(
            f'Hardware Bridge Düğümü Başlatıldı.\n'
            f'  - Motor Portu (Mega): {self.port}\n'
            f'  - Pusula Portu (Uno) : {self.mag_port}'
        )

    def _connect_serials(self):
        # 1. Mega (Motor) Bağlantısı
        try:
            self.ser_mega = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Arduino Mega'ya bağlandı: {self.port} ({self.baudrate} baud)")
        except serial.SerialException as e:
            self.get_logger().error(f"Arduino Mega seri port bağlantı hatası ({self.port}): {e}")
            self.ser_mega = None

        # 2. Uno (Magnetometer) Bağlantısı
        if self.mag_port == self.port:
            self.ser_uno = self.ser_mega
        else:
            try:
                self.ser_uno = serial.Serial(self.mag_port, self.baudrate, timeout=0.1)
                self.get_logger().info(f"Arduino Uno'ya bağlandı: {self.mag_port} ({self.baudrate} baud)")
            except serial.SerialException as e:
                self.get_logger().error(f"Arduino Uno seri port bağlantı hatası ({self.mag_port}): {e}")
                self.ser_uno = None

    # ==================================================
    # MOTOR KOMUTLARI (ROS 2 Twist -> Arduino Mega engine.ino)
    # Yeni Protokol: MOTOR:FWD:PWM, MOTOR:BACK:PWM, MOTOR:LEFT:PWM, MOTOR:RIGHT:PWM, MOTOR:STOP
    # ==================================================
    def _cmd_callback(self, msg: Twist):
        if not self.ser_mega:
            return

        v = msg.linear.x
        w = msg.angular.z

        cmd = "MOTOR:STOP"

        # Dönüş komutu (Açısal hız z)
        if abs(w) > 0.05:
            pwm = self.max_pwm if abs(w) > 0.4 else self.min_pwm
            cmd = f"MOTOR:LEFT:{pwm}" if w > 0 else f"MOTOR:RIGHT:{pwm}"

        # İleri / Geri komutu (Çizgisel hız x)
        elif abs(v) > 0.05:
            pwm = self.max_pwm if abs(v) > 0.4 else self.min_pwm
            cmd = f"MOTOR:FWD:{pwm}" if v > 0 else f"MOTOR:BACK:{pwm}"

        else:
            cmd = "MOTOR:STOP"

        now = time.time()
        self.last_cmd_time = now

        if cmd != self.last_cmd:
            self._send_raw(cmd)
            self.last_cmd = cmd
            self.last_sent_time = now
            self.get_logger().info(f"Arduino Mega Motor Komutu: {cmd}")
        elif cmd != "MOTOR:STOP" and (now - self.last_sent_time) >= 0.2:
            # Arduino Mega 750ms KOMUT_TIMEOUT_MS zaman aşımına sahip.
            # Komut aynı kaldığı sürece 200ms'de bir tazeleyerek gönderiyoruz.
            self._send_raw(cmd)
            self.last_sent_time = now

    def _led_callback(self, msg: String):
        if self.ser_mega:
            mode_cmd = f"LED:{msg.data.upper()}"
            self._send_raw(mode_cmd)

    def _send_raw(self, cmd: str):
        try:
            with self.serial_lock:
                if self.ser_mega and self.ser_mega.is_open:
                    self.ser_mega.write((cmd + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Arduino Mega seri yazma hatası: {e}")

    def _keepalive(self):
        now = time.time()
        if self.last_cmd and self.last_cmd != "MOTOR:STOP":
            if now - self.last_cmd_time > self.motor_watchdog_timeout:
                self.last_cmd = "MOTOR:STOP"
                self._send_raw("MOTOR:STOP")
                self.get_logger().warn("Motor komutu zaman aşımı! Araç durduruldu.")
            elif (now - self.last_sent_time) >= 0.2:
                self._send_raw(self.last_cmd)
                self.last_sent_time = now

    # ==================================================
    # SERİ PORT OKUYUCULAR
    # ==================================================
    def _serial_reader_uno(self):
        buf = ""
        while rclpy.ok():
            if not self.ser_uno or not self.ser_uno.is_open:
                time.sleep(1.0)
                continue

            try:
                waiting = self.ser_uno.in_waiting
                if waiting > 0:
                    chunk = self.ser_uno.read(waiting).decode('ascii', errors='ignore')
                    buf += chunk
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        self._parse_arduino_line(line.strip())
                else:
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f"Arduino Uno seri okuma hatası: {e}")
                time.sleep(0.5)

    def _serial_reader_mega(self):
        buf = ""
        while rclpy.ok():
            if not self.ser_mega or not self.ser_mega.is_open:
                time.sleep(1.0)
                continue

            try:
                waiting = self.ser_mega.in_waiting
                if waiting > 0:
                    with self.serial_lock:
                        chunk = self.ser_mega.read(waiting).decode('ascii', errors='ignore')
                    buf += chunk
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        self._parse_arduino_line(line.strip())
                else:
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f"Arduino Mega seri okuma hatası: {e}")
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
        elif line.startswith("ERR,"):
            self.get_logger().warn(f"Arduino Mega Hata Yanıtı: {line}")
        elif line.startswith("ACK,"):
            self.get_logger().debug(f"Arduino Mega Yanıtı: {line}")

    def destroy_node(self):
        if self.ser_mega and self.ser_mega.is_open:
            self._send_raw("MOTOR:STOP")
            self.ser_mega.close()
        if self.ser_uno and self.ser_uno != self.ser_mega and self.ser_uno.is_open:
            self.ser_uno.close()
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
