#!/usr/bin/env python3
"""
Çoklu/Tekli GPS Test Düğümü (gps_nav_test)
Earendil Bot - Raspberry Pi 5
-----------------------------------------
Parametre olarak girilen 1. ve opsiyonel 2. GPS koordinatına (target_lat/lon ve target2_lat/lon)
sırayla otonom olarak ilerler. Tüm hedefler bitince /mission/status konusuna "COMPLETED" 
mesajı yayınlayarak path_recorder düğümünün üsse geri dönüşünü tetikler.

Kullanım Örnekleri:
  # Tek Hedef:
  ros2 run earendil_bot gps_nav_test --ros-args \
    -p target_lat:=39.9017797 -p target_lon:=32.7704813

  # İki Hedef Sıralı:
  ros2 run earendil_bot gps_nav_test --ros-args \
    -p target_lat:=39.9017797 -p target_lon:=32.7704813 \
    -p target2_lat:=39.9017482 -p target2_lon:=32.7704942
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import math
import time
from earendil_bot.gps.gps_math import bearing_between_gps_rad, haversine, angle_error_rad


class SimpleGpsNavTest(Node):
    def __init__(self):
        super().__init__('gps_nav_test')

        # 1. Hedef Koordinat Parametreleri
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)

        # 2. Hedef Koordinat Parametreleri (Opsiyonel)
        self.declare_parameter('target2_lat', 0.0)
        self.declare_parameter('target2_lon', 0.0)

        # Sürüş Parametreleri
        self.declare_parameter('heading_tolerance_deg', 7.0)
        self.declare_parameter('arrival_radius', 2.0)        # 2 Metre GPS Varış Yarıçapı
        self.declare_parameter('max_linear_x', 0.5)          # m/s
        self.declare_parameter('max_angular_z', 1.0)         # rad/s
        self.declare_parameter('kp_angular', 2.5)            # Dönüş P kazancı
        self.declare_parameter('kp_lane', 1.5)               # İlerleme şerit takip kazancı

        t1_lat = self.get_parameter('target_lat').value
        t1_lon = self.get_parameter('target_lon').value
        t2_lat = self.get_parameter('target2_lat').value
        t2_lon = self.get_parameter('target2_lon').value

        self.targets = []
        if t1_lat != 0.0 or t1_lon != 0.0:
            self.targets.append((t1_lat, t1_lon))
        if t2_lat != 0.0 or t2_lon != 0.0:
            self.targets.append((t2_lat, t2_lon))

        self.current_target_index = 0

        self.heading_tol = math.radians(self.get_parameter('heading_tolerance_deg').value)
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kp_lane = self.get_parameter('kp_lane').value

        # Durum Değişkenleri
        self.current_lat = None
        self.current_lon = None
        self.mag_heading = None

        self.last_mag_time = 0.0
        self.last_gps_time = 0.0
        self.aligned = False
        self.arrived = False

        # Yayıncılar & Aboneler
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)

        # 10 Hz Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"GPS Test Düğümü Başlatıldı. Toplam {len(self.targets)} Hedef Yüklendi.")
        for idx, (lat, lon) in enumerate(self.targets):
            self.get_logger().info(f"  Hedef #{idx + 1}: Lat={lat:.7f}, Lon={lon:.7f}")

    def mag_cb(self, msg: Float32):
        self.mag_heading = math.radians(msg.data)
        self.last_mag_time = time.time()

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_time = time.time()

    def publish_status(self, status_str: str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

    def control_loop(self):
        cmd = Twist()

        if self.arrived:
            self.publish_status("COMPLETED")
            self.stop_robot(cmd)
            return

        self.publish_status("IN_PROGRESS")

        if not self.targets:
            self.get_logger().warn("Hiçbir hedef koordinat girilmedi! Bekleniyor...", throttle_duration_sec=3.0)
            return

        if self.mag_heading is None or (time.time() - self.last_mag_time > 2.0):
            self.get_logger().warn("Pusula verisi bekleniyor (/mag/heading)...", throttle_duration_sec=3.0)
            self.stop_robot(cmd)
            return

        if self.current_lat is None or (time.time() - self.last_gps_time > 2.0):
            self.get_logger().warn("GPS verisi bekleniyor (/gps/fix)...", throttle_duration_sec=3.0)
            self.stop_robot(cmd)
            return

        target_lat, target_lon = self.targets[self.current_target_index]
        distance = haversine(self.current_lat, self.current_lon, target_lat, target_lon)
        target_bearing = bearing_between_gps_rad(self.current_lat, self.current_lon, target_lat, target_lon)

        # Hedefe Varış Kontrolü (2 Metre)
        if distance <= self.arrival_radius:
            self.get_logger().info(f"🎯 HEDEF #{self.current_target_index + 1} ULAŞILDI! Kalan Mesafe: {distance:.2f}m")
            self.stop_robot(cmd)
            
            self.current_target_index += 1
            if self.current_target_index >= len(self.targets):
                self.get_logger().info("🏆 TÜM TEST HEDEFLERİ TAMAMLANDI! Üsse dönüş tetikleniyor...")
                self.publish_status("COMPLETED")
                self.arrived = True
            else:
                next_lat, next_lon = self.targets[self.current_target_index]
                self.get_logger().info(f"🚀 Sıradaki Hedefe Geçiliyor (#2): Lat={next_lat:.7f}, Lon={next_lon:.7f}")
                self.aligned = False
                time.sleep(1.0)
            return

        error = angle_error_rad(target_bearing, self.mag_heading)

        self.get_logger().info(
            f"Hedef #{self.current_target_index + 1} | Mesafe: {distance:.2f}m | Açı Hatası: {math.degrees(error):.1f}° | Durum: {'İLERLİYOR' if self.aligned else 'DÖNÜYOR'}",
            throttle_duration_sec=1.0
        )

        # 1. Aşama: Hedefe Yönelme / Hizalama
        if not self.aligned:
            if abs(error) > self.heading_tol:
                angular_vel = self.kp_angular * error
                angular_vel = max(-self.max_angular_z, min(self.max_angular_z, angular_vel))
                cmd.linear.x = 0.0
                cmd.angular.z = angular_vel
            else:
                self.aligned = True
                self.get_logger().info(f"Hedef #{self.current_target_index + 1} için açı hizalandı! İleri sürüşe geçiliyor.")

        # 2. Aşama: İlerleme ve Şerit Takip
        if self.aligned:
            if abs(error) > self.heading_tol * 3:
                self.aligned = False
                self.get_logger().info("Hizalama bozuldu! Yeniden yöneliniyor.")
            else:
                cmd.linear.x = self.max_linear_x
                cmd.angular.z = self.kp_lane * error
                cmd.angular.z = max(-self.max_angular_z, min(self.max_angular_z, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def stop_robot(self, cmd: Twist):
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGpsNavTest()
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
