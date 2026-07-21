#!/usr/bin/env python3
"""
Komut Kaydı ve Rota Tersleme ile Üsse Dönüş Düğümü (path_recorder_return)
Earendil Bot - Raspberry Pi 5 (MRC 2026 Ekstra Dönüş Puanı Şartnamesi)
-----------------------------------------
1. Gidiş yolunda motora verilen tüm komutları (/cmd_vel) zaman damgaları ve süreleri ile kaydeder.
2. /mission/status konusundan "COMPLETED" (Gidiş Bitti) mesajı geldiğinde:
   a. Pusulayı (/mag/heading) kullanarak robotu tam 180 derece döndürür.
   b. Kaydedilen hareket geçmişini SON VERİLENDEN İLK VERİLENE TERS SIRADA oynatır.
   c. Komutlarda sağları sol, solları sağ yapar (ileri/geri aynı kalır).
3. GPS'e ihtiyaç duymadan aracı aynı rotayı tersten izleterek Ay Üssüne (Lunar Base) ulaştırır.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
import time
import math


class PathRecorderReturnNode(Node):
    def __init__(self):
        super().__init__('path_recorder_return')

        # Durumlar: RECORDING, WAITING_RETURN, TURNING_180, REPLAYING, COMPLETED
        self.state = "RECORDING"

        # Komut geçmişi: [(cmd_string, duration_seconds), ...]
        self.history = []
        self.current_cmd = "dur"
        self.current_cmd_start = time.time()

        self.mag_heading = None
        self.initial_turn_target = None

        # Yayıncı ve Aboneler
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.led_pub = self.create_publisher(String, '/mode/led', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(String, '/mission/status', self.status_cb, 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)

        # 10 Hz Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)

        # Replay döngü değişkenleri
        self.replay_index = 0
        self.replay_cmd_start = 0.0

        self.get_logger().info("Komut Kaydı ve Rota Tersleme Düğümü Başlatıldı. Gidiş hareketleri kaydediliyor...")

    def mag_cb(self, msg: Float32):
        self.mag_heading = math.radians(msg.data)

    def cmd_cb(self, msg: Twist):
        if self.state != "RECORDING":
            return

        # Twist hızından komut dizesi belirleme
        v = msg.linear.x
        w = msg.angular.z

        cmd_str = "dur"
        if abs(w) > 0.05:
            suffix = "hizli" if abs(w) > 0.4 else "yavas"
            cmd_str = f"sol_{suffix}" if w > 0 else f"sag_{suffix}"
        elif abs(v) > 0.05:
            suffix = "hizli" if abs(v) > 0.4 else "yavas"
            cmd_str = f"ileri_{suffix}" if v > 0 else f"geri_{suffix}"

        now = time.time()
        duration = now - self.current_cmd_start

        if cmd_str != self.current_cmd:
            if self.current_cmd != "dur" and duration > 0.1:
                self.history.append((self.current_cmd, duration))
                self.get_logger().info(f"[KAYIT] Komut: {self.current_cmd} | Süre: {duration:.2f}s")
            self.current_cmd = cmd_str
            self.current_cmd_start = now

    def status_cb(self, msg: String):
        if msg.data == "COMPLETED" and self.state in ["RECORDING", "WAITING_RETURN"]:
            # Son hareket varsa kayda ekle
            duration = time.time() - self.current_cmd_start
            if self.current_cmd != "dur" and duration > 0.1:
                self.history.append((self.current_cmd, duration))

            self.get_logger().info(f"🏁 TÜM GİDİŞ TAMAMLANDI! Toplam {len(self.history)} hareket kaydedildi.")
            self.get_logger().info("Üsse Dönüş Başlatılıyor: 1. Aşama 180° Dönüş yapılıyor...")
            self.state = "TURNING_180"
            if self.mag_heading is not None:
                # 180 derece ters açı hedefi
                self.initial_turn_target = (self.mag_heading + math.pi) % (2 * math.pi)

    def invert_command(self, cmd_str: str) -> Twist:
        """Sağ komutlarını Sol, Sol komutlarını Sağ yapar. Twist hızına çevirir."""
        cmd = Twist()
        if cmd_str.startswith("sag_"):
            is_fast = "hizli" in cmd_str
            cmd.angular.z = 0.6 if is_fast else 0.3  # Sağın tersi SOL (+w)
        elif cmd_str.startswith("sol_"):
            is_fast = "hizli" in cmd_str
            cmd.angular.z = -0.6 if is_fast else -0.3 # Solun tersi SAĞ (-w)
        elif cmd_str.startswith("ileri_"):
            is_fast = "hizli" in cmd_str
            cmd.linear.x = 0.5 if is_fast else 0.2
        elif cmd_str.startswith("geri_"):
            is_fast = "hizli" in cmd_str
            cmd.linear.x = -0.5 if is_fast else -0.2
        return cmd

    def control_loop(self):
        cmd = Twist()

        if self.state in ["RECORDING", "WAITING_RETURN"]:
            return

        if self.state == "COMPLETED":
            self.stop_robot(cmd)
            return

        # ---------------------------------------------------------
        # 1. AŞAMA: 180 Derece Dönüş (Pusula İle)
        # ---------------------------------------------------------
        if self.state == "TURNING_180":
            if self.mag_heading is None:
                self.get_logger().warn("180° dönüş için pusula verisi bekleniyor...", throttle_duration_sec=2.0)
                self.stop_robot(cmd)
                return

            if self.initial_turn_target is None:
                self.initial_turn_target = (self.mag_heading + math.pi) % (2 * math.pi)

            # Açı hatası hesaplama
            diff = self.initial_turn_target - self.mag_heading
            diff = (diff + math.pi) % (2 * math.pi) - math.pi

            if abs(diff) > math.radians(7.0):
                cmd.angular.z = 0.5 if diff > 0 else -0.5
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info("180° Dönüş Tamamlandı! 2. Aşama: Ters Rota Oynatılıyor...")
                self.stop_robot(cmd)
                self.state = "REPLAYING"
                self.replay_index = len(self.history) - 1
                self.replay_cmd_start = time.time()
            return

        # ---------------------------------------------------------
        # 2. AŞAMA: Kaydedilen Rotaları Tersten Oynatma
        # ---------------------------------------------------------
        if self.state == "REPLAYING":
            if self.replay_index < 0:
                self.get_logger().info("🎉 ÜSSE GERİ DÖNÜŞ BAŞARIYLA TAMAMLANDI! Robot Ay Üssünde.")
                self.stop_robot(cmd)
                self.state = "COMPLETED"
                return

            orig_cmd, orig_duration = self.history[self.replay_index]
            elapsed = time.time() - self.replay_cmd_start

            if elapsed < orig_duration:
                inverted_twist = self.invert_command(orig_cmd)
                self.cmd_pub.publish(inverted_twist)
                self.get_logger().info(
                    f"Geri Dönüş [{len(self.history) - self.replay_index}/{len(self.history)}]: "
                    f"Orijinal: {orig_cmd} -> Uygulanan: {orig_cmd.replace('sag', 'TEMP').replace('sol', 'sag').replace('TEMP', 'sol')} | "
                    f"Kalan Süre: {(orig_duration - elapsed):.1f}s",
                    throttle_duration_sec=1.0
                )
            else:
                self.stop_robot(cmd)
                self.replay_index -= 1
                self.replay_cmd_start = time.time()

    def stop_robot(self, cmd: Twist):
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderReturnNode()
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
