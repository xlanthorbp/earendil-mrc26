#!/usr/bin/env python3
"""
Zamanlı Navigasyon Test Düğümü (timed_nav_test)
Earendil Bot - Raspberry Pi 5 (MRC 2026)
-----------------------------------------
Belirlenen sürelerde sıralı hareketler gerçekleştirir:
1. 5 saniye İleri
2. 2 saniye Sağa Dönüş
3. 2 saniye İleri
4. 1 saniye Sağa Dönüş

Hareket dizisi tamamlandığında /mission/status konusuna "COMPLETED" yayınlar.
path_recorder düğümü bu sinyali alarak 180° dönüp aracı aynı rotadan geri getirir.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TimedNavTestNode(Node):
    def __init__(self):
        super().__init__('timed_nav_test')

        # Parametreler (Varsayılan süre ve hızlar)
        self.declare_parameter('forward_time_1', 5.0)  # 1. İleri süre [saniye]
        self.declare_parameter('turn_time_1', 2.0)     # 1. Sağa dönüş süre [saniye]
        self.declare_parameter('forward_time_2', 2.0)  # 2. İleri süre [saniye]
        self.declare_parameter('turn_time_2', 1.0)     # 2. Sağa dönüş süre [saniye]

        self.declare_parameter('linear_speed', 0.5)    # İleri hız [m/s]
        self.declare_parameter('angular_speed', 0.5)   # Dönüş hızı [rad/s] (Sağa dönüş için negative)

        self.fwd_t1 = self.get_parameter('forward_time_1').value
        self.turn_t1 = self.get_parameter('turn_time_1').value
        self.fwd_t2 = self.get_parameter('forward_time_2').value
        self.turn_t2 = self.get_parameter('turn_time_2').value

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Hareket Adımları: (Adım Adı, Süre [sn], Linear_X, Angular_Z)
        self.steps = [
            ("1. İleri Gidiş (5s)", self.fwd_t1, self.linear_speed, 0.0),
            ("2. Sağa Dönüş (2s)", self.turn_t1, 0.0, -self.angular_speed),
            ("3. İleri Gidiş (2s)", self.fwd_t2, self.linear_speed, 0.0),
            ("4. Sağa Dönüş (1s)", self.turn_t2, 0.0, -self.angular_speed),
        ]

        self.current_step_idx = 0
        self.step_start_time = time.time()
        self.completed_sent = False

        # Yayıncılar
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # 10 Hz Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🚀 Zamanlı Navigasyon Test Düğümü Başlatıldı!")
        self.get_logger().info(f"  - 1. Aşama: {self.fwd_t1}s İleri")
        self.get_logger().info(f"  - 2. Aşama: {self.turn_t1}s Sağa Dönüş")
        self.get_logger().info(f"  - 3. Aşama: {self.fwd_t2}s İleri")
        self.get_logger().info(f"  - 4. Aşama: {self.turn_t2}s Sağa Dönüş")

    def control_loop(self):
        if self.current_step_idx >= len(self.steps):
            if not self.completed_sent:
                # Durma komutu ve COMPLETED sinyali
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

                status_msg = String()
                status_msg.data = "COMPLETED"
                self.status_pub.publish(status_msg)

                self.get_logger().info("🏁 TÜM HAREKETLER TAMAMLANDI! '/mission/status' -> 'COMPLETED' yayınlandı.")
                self.completed_sent = True

            return

        step_name, step_duration, linear_x, angular_z = self.steps[self.current_step_idx]
        elapsed = time.time() - self.step_start_time

        if elapsed < step_duration:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            self.cmd_pub.publish(cmd)

            remaining = step_duration - elapsed
            self.get_logger().info(
                f"[{step_name}] İlerleme: {elapsed:.1f}s / {step_duration:.1f}s (Kalan: {remaining:.1f}s)",
                throttle_duration_sec=1.0
            )
        else:
            # Mevcut adım bitti, sonraki adıma geç
            self.get_logger().info(f"✅ Adım Tamamlandı: {step_name}")
            self.current_step_idx += 1
            self.step_start_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = TimedNavTestNode()
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
