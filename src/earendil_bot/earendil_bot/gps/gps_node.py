#!/usr/bin/env python3
"""
Standart Tekli GPS ROS 2 Düğümü (gps_node)
Earendil Bot - Raspberry Pi 5
-----------------------------------------
Raspberry Pi 5'e USB/Seri port üzerinden bağlanan GPS alıcısından gelen verileri
doğrudan ROS 2 konularına (topics) yayınlar:
  1. /gps/fix       -> NavSatFix (Enlem, Boylam, Rakım)
  2. /gps/map_link  -> String (Google Harita Bağlantısı)
  3. /gps/raw_nmea  -> String (Ham NMEA Cümleciği)
"""

import serial
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


def parse_nmea_lat_lon(lat_str: str, lat_dir: str, lon_str: str, lon_dir: str):
    """NMEA DDMM.MMMM formatını ondalık dereceye (decimal degrees) dönüştürür."""
    if not lat_str or not lon_str or not lat_dir or not lon_dir:
        return None, None
    try:
        # Enlem: DDMM.MMMM
        lat_deg = float(lat_str[:2])
        lat_min = float(lat_str[2:])
        lat = lat_deg + (lat_min / 60.0)
        if lat_dir == 'S':
            lat = -lat

        # Boylam: DDDMM.MMMM
        lon_deg = float(lon_str[:3])
        lon_min = float(lon_str[3:])
        lon = lon_deg + (lon_min / 60.0)
        if lon_dir == 'W':
            lon = -lon

        return lat, lon
    except (ValueError, IndexError):
        return None, None


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Parametreler
        self.declare_parameter('gps_port', '/dev/ttyUSB1')
        self.declare_parameter('gps_baud', 115200)
        self.declare_parameter('map_link_print_interval', 3.0)

        self.gps_port = self.get_parameter('gps_port').value
        self.gps_baud = self.get_parameter('gps_baud').value
        self.map_link_print_interval = self.get_parameter('map_link_print_interval').value

        # Yayıncılar (Publishers)
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.map_link_pub = self.create_publisher(String, '/gps/map_link', 10)
        self.raw_nmea_pub = self.create_publisher(String, '/gps/raw_nmea', 10)

        self.get_logger().info(f"GPS Düğümü Başlatılıyor... Port: {self.gps_port}, Baud: {self.gps_baud}")

        # Seri port ve okuma döngüsü
        self.running = True
        self.serial_conn = None
        self.last_map_print_time = 0.0

        self.thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.thread.start()

    def read_serial_loop(self):
        while self.running and rclpy.ok():
            try:
                if self.serial_conn is None or not self.serial_conn.is_open:
                    self.get_logger().info(f"GPS seri portuna bağlanılıyor ({self.gps_port})...")
                    self.serial_conn = serial.Serial(self.gps_port, self.gps_baud, timeout=1.0)
                    self.get_logger().info("GPS Seri Bağlantısı Başarılı!")

                line_bytes = self.serial_conn.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode('ascii', errors='ignore').strip()
                if not line.startswith('$'):
                    continue

                # Ham NMEA cümlesini direkt ROS 2 konusuna yaz
                raw_msg = String()
                raw_msg.data = line
                self.raw_nmea_pub.publish(raw_msg)

                # NMEA ayrıştırma ve NavSatFix yayınlama
                self.process_nmea_line(line)

            except serial.SerialException as e:
                self.get_logger().error(f"GPS Seri Port Hatası: {e}. 2s sonra tekrar deneniyor...")
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                self.serial_conn = None
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().error(f"GPS Beklenmeyen Hata: {e}")
                time.sleep(1.0)

    def process_nmea_line(self, line: str):
        parts = line.split(',')
        if len(parts) < 10:
            return

        msg_type = parts[0]
        lat, lon = None, None

        if msg_type.endswith('GGA'):
            lat_str, lat_dir = parts[2], parts[3]
            lon_str, lon_dir = parts[4], parts[5]
            fix_quality = parts[6] if len(parts) > 6 else '0'
            if fix_quality != '0':
                lat, lon = parse_nmea_lat_lon(lat_str, lat_dir, lon_str, lon_dir)

        elif msg_type.endswith('RMC'):
            status = parts[2] if len(parts) > 2 else 'V'
            if status == 'A':
                lat_str, lat_dir = parts[3], parts[4]
                lon_str, lon_dir = parts[5], parts[6]
                lat, lon = parse_nmea_lat_lon(lat_str, lat_dir, lon_str, lon_dir)

        if lat is not None and lon is not None:
            # NavSatFix Mesajı Yayınla
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = 0.0
            self.fix_pub.publish(msg)

            # Harita Bağlantısı Yayınla
            now = time.time()
            if now - self.last_map_print_time >= self.map_link_print_interval:
                map_url = f"https://www.google.com/maps?q={lat:.7f},{lon:.7f}"
                link_msg = String()
                link_msg.data = map_url
                self.map_link_pub.publish(link_msg)
                self.get_logger().info(f"GPS Konumu: Lat={lat:.7f}, Lon={lon:.7f} | {map_url}")
                self.last_map_print_time = now

    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
