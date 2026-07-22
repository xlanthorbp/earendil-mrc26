#!/usr/bin/env python3
"""
MRC 2026 Ana Görev Navigasyon Düğümü (gps_nav_main)
Earendil Bot - Raspberry Pi 5
-----------------------------------------
1. Sırasıyla 10 GPS hedefine ilerler (GPS Varış Hassasiyeti: 2 Metre).
2. Seyir Esnasında Görsel ArUco Override:
   - Yolda X hedefine giderken kamera ArUco etiketini görürse (10m içinde),
     GPS navigasyonunu yarıda keser ve doğrudan ArUco etiketine yönelip 50 cm yaklaşır.
3. Hedefe Ulaşınca 360° Tarama:
   - GPS hedefine 2 metre yaklaşıldığında yolda ArUco görülmediyse, araç yavaşça 360° döner.
   - ArUco algılandığı an dönüşü keser, etiket yönüne 50 cm yaklaşır.
4. 50 cm'ye ulaşılınca:
   - Sarı LED moduna geçer (Sensing).
   - O anki koordinatı ve ArUco ID'sini kaydeder.
5. Tüm hedefler bitince:
   - /mission/status konusuna "COMPLETED" yayınlar (path_recorder düğümünün üsse dönmesi için).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Int32, Bool
from geometry_msgs.msg import Point
import math
import time
import csv
from pathlib import Path
from earendil_bot.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class GpsNavMainNode(Node):
    def __init__(self):
        super().__init__('gps_nav_main')

        # Sürüş ve Algılama Parametreleri
        self.declare_parameter('heading_tolerance_deg', 7.0)
        self.declare_parameter('gps_arrival_radius', 2.0)    # 2 Metre GPS varış yarıçapı
        self.declare_parameter('aruco_arrival_radius', 0.5)  # 50 cm ArUco etiketine yaklaşma yarıçapı
        self.declare_parameter('max_linear_x', 0.5)          # m/s
        self.declare_parameter('max_angular_z', 1.0)         # rad/s
        self.declare_parameter('kp_angular', 2.5)
        self.declare_parameter('kp_lane', 1.5)
        self.declare_parameter('kp_aruco_steer', 0.05)       # ArUco açısal takip kazancı
        self.declare_parameter('sensing_wait_time', 5.0)     # Algılama bekleme süresi

        self.heading_tol = math.radians(self.get_parameter('heading_tolerance_deg').value)
        self.gps_arrival_radius = self.get_parameter('gps_arrival_radius').value
        self.aruco_arrival_radius = self.get_parameter('aruco_arrival_radius').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kp_lane = self.get_parameter('kp_lane').value
        self.kp_aruco_steer = self.get_parameter('kp_aruco_steer').value
        self.sensing_wait_time = self.get_parameter('sensing_wait_time').value

        # 10 GPS Hedef Listesi (Örnek Şartname Koordinatları)
        self.waypoints = [
            (1, 39.9017797, 32.7704813),
            (2, 39.9017482, 32.7704942),
            (3, 39.9017200, 32.7705100),
            (4, 39.9017000, 32.7705300),
            (5, 39.9016800, 32.7705500),
            (6, 39.9016600, 32.7705700),
            (7, 39.9016400, 32.7705900),
            (8, 39.9016200, 32.7706100),
            (9, 39.9016000, 32.7706300),
            (10, 39.9015800, 32.7706500),
        ]

        self.current_wp_index = 0

        # Durumlar: NAVIGATING_GPS, OVERRIDE_ARUCO, SCANNING_360, SENSING_RECORD, MISSION_COMPLETED
        self.state = "NAVIGATING_GPS"
        self.sensing_start_time = 0.0
        self.scan_360_start_time = 0.0

        # Sensör Verileri
        self.current_lat = None
        self.current_lon = None
        self.mag_heading = None

        self.aruco_visible = False
        self.aruco_id = -1
        self.aruco_angle_deg = 0.0
        self.aruco_dist_m = 999.0

        self.last_mag_time = 0.0
        self.last_gps_time = 0.0
        self.aligned = False

        # Yayıncılar & Aboneler
        self.cmd_pub = self.create_publisher(String, '/motor/command', 10)
        self.led_pub = self.create_publisher(String, '/mode/led', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Bool, '/aruco_visible', self.aruco_visible_cb, 10)
        self.create_subscription(Int32, '/aruco_id', self.aruco_id_cb, 10)
        self.create_subscription(Point, '/aruco_pose', self.aruco_pose_cb, 10)

        # CSV Kayıt Dosyası
        self.log_file_path = Path.home() / "mrc_waypoints_recorded.csv"
        self.init_csv_log()

        # 10 Hz Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"MRC 2026 Ana Görev Düğümü (gps_nav_main) Başlatıldı. GPS Hassasiyeti: {self.gps_arrival_radius}m")

    def init_csv_log(self):
        try:
            with open(self.log_file_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Waypoint_Seq", "Latitude", "Longitude", "Heading_Deg", "Detected_ArUco_ID", "Detection_Type"])
        except Exception as e:
            self.get_logger().error(f"CSV kayıt dosyası hatası: {e}")

    def save_waypoint_record(self, detection_type: str):
        try:
            with open(self.log_file_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                heading_deg = math.degrees(self.mag_heading) if self.mag_heading else 0.0
                seq = self.waypoints[self.current_wp_index][0]
                writer.writerow([
                    seq,
                    self.current_lat or 0.0,
                    self.current_lon or 0.0,
                    f"{heading_deg:.2f}",
                    self.aruco_id,
                    detection_type
                ])
                self.get_logger().info(f"KAYIT EDİLDİ -> WP #{seq} | Lat: {self.current_lat:.7f}, Lon: {self.current_lon:.7f} | ArUco ID: {self.aruco_id} | Tip: {detection_type}")
        except Exception as e:
            self.get_logger().error(f"Kayıt ekleme hatası: {e}")

    def mag_cb(self, msg: Float32):
        self.mag_heading = math.radians(msg.data)
        self.last_mag_time = time.time()

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_time = time.time()

    def aruco_visible_cb(self, msg: Bool):
        self.aruco_visible = msg.data

    def aruco_id_cb(self, msg: Int32):
        self.aruco_id = msg.data

    def aruco_pose_cb(self, msg: Point):
        self.aruco_angle_deg = msg.x
        self.aruco_dist_m = msg.z

    def set_led_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.led_pub.publish(msg)

    def publish_status(self, status_str: str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

    def control_loop(self):
        cmd = String()

        if self.state == "MISSION_COMPLETED":
            self.set_led_mode("GREEN")
            self.publish_status("COMPLETED")
            self.stop_robot()
            return

        self.publish_status("IN_PROGRESS")

        if self.mag_heading is None or (time.time() - self.last_mag_time > 2.0):
            self.get_logger().warn("Pusula verisi bekleniyor (/mag/heading)...", throttle_duration_sec=3.0)
            self.stop_robot()
            return

        if self.current_lat is None or (time.time() - self.last_gps_time > 2.0):
            self.get_logger().warn("GPS verisi bekleniyor (/gps/fix)...", throttle_duration_sec=3.0)
            self.stop_robot()
            return

        seq, target_lat, target_lon = self.waypoints[self.current_wp_index]
        distance_to_gps = haversine(self.current_lat, self.current_lon, target_lat, target_lon)
        target_bearing = bearing_between_gps_rad(self.current_lat, self.current_lon, target_lat, target_lon)

        # ---------------------------------------------------------
        # 1. GÖRSEL OVERRIDE KONTROLÜ (Seyir Esnasında ArUco Algılanması)
        # ---------------------------------------------------------
        if self.state == "NAVIGATING_GPS" and self.aruco_visible and self.aruco_dist_m < 10.0:
            self.get_logger().info(f"⚡ GÖRSEL OVERRIDE! ArUco ID={self.aruco_id} algılandı (Mesafe: {self.aruco_dist_m:.2f}m). ArUco'ya yöneliniyor!")
            self.state = "OVERRIDE_ARUCO"

        # ---------------------------------------------------------
        # 2. OVERRIDE_ARUCO (Doğrudan ArUco Etiketine Doğru Sürüş)
        # ---------------------------------------------------------
        if self.state == "OVERRIDE_ARUCO":
            self.set_led_mode("RED")

            # ArUco etiketine 50 cm yaklaşıldı mı?
            if self.aruco_dist_m <= self.aruco_arrival_radius:
                self.get_logger().info(f"🎯 ARUCO ETİKETİNE 50 CM ULAŞILDI! ID={self.aruco_id}")
                self.stop_robot()
                self.save_waypoint_record(detection_type="VISUAL_OVERRIDE")
                self.state = "SENSING_RECORD"
                self.sensing_start_time = time.time()
                return

            # Kamera ile hizalanıp etikete doğru sürüş
            if abs(self.aruco_angle_deg) > 5.0:
                cmd.data = "MOTOR:LEFT:80" if self.aruco_angle_deg > 0 else "MOTOR:RIGHT:80"
            else:
                cmd.data = "MOTOR:FWD:80"
            self.cmd_pub.publish(cmd)
            return

        # ---------------------------------------------------------
        # 3. SCANNING_360 (Hedefe Ulaşılıp ArUco Görülmediğinde 360° Dönüş)
        # ---------------------------------------------------------
        if self.state == "SCANNING_360":
            self.set_led_mode("RED")

            if self.aruco_visible:
                self.get_logger().info(f"360° TARAMADA ARUCO BULUNDU! ID={self.aruco_id}. ArUco'ya ilerleniyor.")
                self.state = "OVERRIDE_ARUCO"
                return

            elapsed_scan = time.time() - self.scan_360_start_time
            if elapsed_scan < 12.0:  # ~360° dönüş
                cmd.data = "MOTOR:LEFT:80"
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().warn(f"WP #{seq} için 360° taramada ArUco bulunamadı. GPS konumu kaydediliyor.")
                self.save_waypoint_record(detection_type="GPS_ONLY")
                self.state = "SENSING_RECORD"
                self.sensing_start_time = time.time()
            return

        # ---------------------------------------------------------
        # 4. SENSING_RECORD (Sarı Işık Bekleme Modu)
        # ---------------------------------------------------------
        if self.state == "SENSING_RECORD":
            self.set_led_mode("YELLOW")
            self.stop_robot()

            if time.time() - self.sensing_start_time >= self.sensing_wait_time:
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    self.get_logger().info("🏆 TÜM 10 HEDEF BAŞARIYLA TAMAMLANDI! Üsse dönüş tetikleniyor.")
                    self.state = "MISSION_COMPLETED"
                else:
                    self.get_logger().info(f"Sıradaki Hedefe Geçiliyor: Waypoint #{self.waypoints[self.current_wp_index][0]}")
                    self.state = "NAVIGATING_GPS"
                    self.aligned = False
            return

        # ---------------------------------------------------------
        # 5. NAVIGATING_GPS (Standart GPS Hedefine Sürüş)
        # ---------------------------------------------------------
        self.set_led_mode("RED")

        # GPS Hedefine 2 Metre yaklaşıldı mı?
        if distance_to_gps <= self.gps_arrival_radius:
            self.get_logger().info(f"GPS HEDEFİNE 2 METRE YAKLAŞILDI! Waypoint #{seq}. 360° ArUco Taraması Başlatılıyor...")
            self.stop_robot()
            self.state = "SCANNING_360"
            self.scan_360_start_time = time.time()
            return

        # Yönelim Hatası
        error = angle_error_rad(target_bearing, self.mag_heading)

        # Dönüş / Yönelme
        if not self.aligned:
            if abs(error) > self.heading_tol:
                cmd.data = "MOTOR:LEFT:80" if error > 0 else "MOTOR:RIGHT:80"
            else:
                self.aligned = True

        # İlerleme ve Şerit Takip
        if self.aligned:
            if abs(error) > self.heading_tol * 3:
                self.aligned = False
                cmd.data = "MOTOR:LEFT:80" if error > 0 else "MOTOR:RIGHT:80"
            else:
                cmd.data = "MOTOR:FWD:80"

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = String()
        cmd.data = "MOTOR:STOP"
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GpsNavMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
