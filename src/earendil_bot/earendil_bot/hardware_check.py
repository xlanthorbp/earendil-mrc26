#!/usr/bin/env python3
"""
Donanım Durumu Kontrol Düğümü (hardware_check)
Earendil Bot - Raspberry Pi 5 (MRC 2026)
-----------------------------------------
Aktif sensör ve sistem konularını (topics) dinleyerek donanımlardan canlı veri
akıp akmadığını 1 saniyede bir terminal ekranında yeşil/kırmızı göstergelerle kontrol eder.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Bool


class HardwareCheckerNode(Node):
    def __init__(self):
        super().__init__('hardware_checker')

        # Sensör Veri Zaman Damgaları
        self.last_msg_times = {
            'GPS (/gps/fix)': 0.0,
            'Manyetometre (/mag/heading)': 0.0,
            'Kamera ArUco (/aruco_visible)': 0.0,
        }

        # Aboneler
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)
        self.create_subscription(Bool, '/aruco_visible', self.cam_cb, 10)

        # 1 saniyelik kontrol zamanlayıcısı
        self.create_timer(1.0, self.print_status)
        self.get_logger().info('Donanım kontrol düğümü başlatıldı. Sensörler dinleniyor...\n')

    def gps_cb(self, msg):
        self.last_msg_times['GPS (/gps/fix)'] = self.get_clock().now().nanoseconds / 1e9

    def mag_cb(self, msg):
        self.last_msg_times['Manyetometre (/mag/heading)'] = self.get_clock().now().nanoseconds / 1e9

    def cam_cb(self, msg):
        self.last_msg_times['Kamera ArUco (/aruco_visible)'] = self.get_clock().now().nanoseconds / 1e9

    def print_status(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        print("\033[H\033[J", end="")  # Ekranı temizle
        print("=" * 55)
        print(" EARENDIL BOT - DONANIM DURUM KONTROLÜ ".center(55, "="))
        print("=" * 55)

        all_ok = True

        for name, last_time in self.last_msg_times.items():
            if last_time == 0.0:
                print(f"[\033[91m HATA \033[0m] {name:32} -> Veri alınamadı!")
                all_ok = False
            else:
                elapsed = current_time - last_time
                if elapsed > 2.0:
                    print(f"[\033[93m UYARI \033[0m] {name:32} -> Veri akışı durdu ({elapsed:.1f}s önce)")
                    all_ok = False
                else:
                    print(f"[\033[92m AKTİF \033[0m] {name:32} -> Gecikme: {elapsed:.2f}s")

        print("=" * 55)
        if all_ok:
            print(" SONUÇ: Tüm sensörler ve donanımlar sorunsuz çalışıyor! \n")
        else:
            print(" SONUÇ: Bazı sensörlerde bağlantı kesintisi veya veri kaybı var! \n")


def main(args=None):
    rclpy.init(args=args)
    node = HardwareCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
