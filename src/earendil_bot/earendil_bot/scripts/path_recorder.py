#!/usr/bin/env python3
"""
Komut Kaydı ve Rota Tersleme ile Üsse Dönüş Düğümü (path_recorder_return)
Earendil Bot - Raspberry Pi 5 (MRC 2026 Ekstra Dönüş Puanı Şartnamesi)
-----------------------------------------
1. Gidiş yolunda motora verilen tüm komutları (/motor/command) zaman damgaları ve süreleri ile kaydeder.
2. Rotayı diske JSON olarak saklar (~/.ros/earendil_bot/last_route.json).
3. /mission/status konusundan "COMPLETED" (Gidiş Bitti) mesajı geldiğinde:
   a. PREPARING_TURN: Aracı 0.4s durdurup dairesel ortalama (circular_mean_deg) ile durduğu pusula açısını sabitler.
   b. TURNING_180: Pusulayı kullanarak aracı tam 180 derece döndürür (ardışık 3 tolerans okuma doğrulaması ile).
   c. RETURN_PAUSE: Dönüş sonrası 0.5s duraklayıp aracı sönümlendirir.
   d. REPLAYING: Kaydedilen hareket geçmişini SON VERİLENDEN İLK VERİLENE TERS SIRADA oynatır.
4. GPS'e ihtiyaç duymadan aracı aynı rotayı tersten izleterek Ay Üssüne (Lunar Base) ulaştırır.
"""

from collections import deque
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from earendil_bot.gps.gps_math import (
    circular_mean_deg,
    normalize_heading_deg,
    shortest_angular_error_deg,
)


class PathRecorderReturnNode(Node):
    def __init__(self):
        super().__init__('path_recorder_return')

        # Durumlar: RECORDING, PREPARING_TURN, TURNING_180, RETURN_PAUSE, REPLAYING, COMPLETED
        self.state = "RECORDING"

        # Komut geçmişi: [(cmd_string, duration_seconds), ...]
        self.history = []
        self.current_cmd = "MOTOR:STOP"
        self.current_cmd_start = time.time()

        # Pusula Dairesel Tamponu (Son 10 Ölçüm)
        self.recent_headings_deg = deque(maxlen=10)
        self.last_heading_time = 0.0
        self.mag_heading_deg = None

        # Dönüş Parametreleri
        self.target_heading_deg = None
        self.turn_settle_until = 0.0
        self.heading_samples_in_tolerance = 0

        self.declare_parameter('motor_cmd_topic', '/motor/command')
        self.declare_parameter('heading_tolerance_deg', 7.5)
        self.declare_parameter('turn_initial_settle_s', 0.4)
        self.declare_parameter('turn_required_samples', 3)
        self.declare_parameter('post_turn_pause_s', 0.5)
        self.declare_parameter('route_log_path', '~/.ros/earendil_bot/last_route.json')

        self.cmd_topic = self.get_parameter('motor_cmd_topic').value
        self.heading_tolerance = float(self.get_parameter('heading_tolerance_deg').value)
        self.turn_initial_settle = float(self.get_parameter('turn_initial_settle_s').value)
        self.turn_required_samples = int(self.get_parameter('turn_required_samples').value)
        self.post_turn_pause = float(self.get_parameter('post_turn_pause_s').value)
        self.route_log_path = os.path.expanduser(str(self.get_parameter('route_log_path').value))

        # Yayıncı ve Aboneler
        self.cmd_pub = self.create_publisher(String, self.cmd_topic, 10)
        self.led_pub = self.create_publisher(String, '/mode/led', 10)

        self.create_subscription(String, self.cmd_topic, self.cmd_cb, 10)
        self.create_subscription(String, '/mission/status', self.status_cb, 10)
        self.create_subscription(Float32, '/mag/heading', self.mag_cb, 10)

        # 10 Hz Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)

        # Replay döngü değişkenleri
        self.replay_index = 0
        self.replay_cmd_start = 0.0
        self.pause_deadline = 0.0

        self.get_logger().info("Komut Kaydı ve Rota Tersleme Düğümü Başlatıldı. Gidiş hareketleri kaydediliyor...")

    def mag_cb(self, msg: Float32):
        deg = normalize_heading_deg(float(msg.data))
        self.mag_heading_deg = deg
        self.last_heading_time = time.time()
        self.recent_headings_deg.append(deg)

    def cmd_cb(self, msg: String):
        if self.state != "RECORDING":
            return

        cmd_str = msg.data.strip().upper()
        now = time.time()
        duration = now - self.current_cmd_start

        if cmd_str != self.current_cmd:
            if self.current_cmd != "MOTOR:STOP" and duration > 0.1:
                self.history.append((self.current_cmd, duration))
                self.get_logger().info(f"[KAYIT] Komut: {self.current_cmd} | Süre: {duration:.2f}s")
            self.current_cmd = cmd_str
            self.current_cmd_start = now

    def status_cb(self, msg: String):
        if msg.data == "COMPLETED" and self.state == "RECORDING":
            # Son hareket varsa kayda ekle
            duration = time.time() - self.current_cmd_start
            if self.current_cmd != "MOTOR:STOP" and duration > 0.1:
                self.history.append((self.current_cmd, duration))

            self.get_logger().info(f"🏁 TÜM GİDİŞ TAMAMLANDI! Toplam {len(self.history)} hareket kaydedildi.")
            self.save_route_json()

            self.state = "PREPARING_TURN"
            self.turn_settle_until = time.time() + self.turn_initial_settle
            self.stop_robot()
            self.get_logger().info(f"Üsse Dönüş Başlatılıyor: 1. Aşama Sönümlenme bekleniyor ({self.turn_initial_settle}s)...")

    def invert_command(self, cmd_str: str) -> str:
        """Sağ komutlarını Sol, Sol komutlarını Sağ yapar. PWM değerini korur."""
        if "LEFT" in cmd_str:
            return cmd_str.replace("LEFT", "RIGHT")
        elif "RIGHT" in cmd_str:
            return cmd_str.replace("RIGHT", "LEFT")
        return cmd_str

    def save_route_json(self):
        """Kaydedilen tüm rotayı JSON dosyası olarak diske yazar."""
        try:
            os.makedirs(os.path.dirname(self.route_log_path), exist_ok=True)
            payload = []
            for cmd, dur in self.history:
                payload.append({
                    "command": cmd,
                    "duration_s": round(dur, 3),
                    "return_command": self.invert_command(cmd)
                })
            with open(self.route_log_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"💾 Rota başarıyla JSON dosyasına kaydedildi: {self.route_log_path}")
        except Exception as e:
            self.get_logger().error(f"Rota JSON dosyasına kaydedilemedi: {e}")

    def control_loop(self):
        if self.state in ["RECORDING", "WAITING_RETURN"]:
            return

        if self.state == "COMPLETED":
            self.stop_robot()
            return

        now = time.time()

        # ---------------------------------------------------------
        # 1. AŞAMA: Dairesel Ortalama Açı Hesabı ile Hazırlık
        # ---------------------------------------------------------
        if self.state == "PREPARING_TURN":
            self.stop_robot()
            if now < self.turn_settle_until:
                return

            if not self.recent_headings_deg or (now - self.last_heading_time > 1.5):
                self.get_logger().warn("180° dönüş için pusula verisi bekleniyor...", throttle_duration_sec=2.0)
                return

            # Dairesel ortalama ile durduğu andaki pusula açısını hesapla
            baseline_heading = circular_mean_deg(self.recent_headings_deg)
            self.target_heading_deg = normalize_heading_deg(baseline_heading + 180.0)
            self.heading_samples_in_tolerance = 0
            self.state = "TURNING_180"
            self.get_logger().info(
                f"2. Aşama: 180° Dönüş Başladı. Başlangıç Açısı: {baseline_heading:.1f}° | Hedef Açı: {self.target_heading_deg:.1f}°"
            )
            return

        # ---------------------------------------------------------
        # 2. AŞAMA: 180 Derece Dönüş (Ardışık 3 Örnek Doğrulaması İle)
        # ---------------------------------------------------------
        if self.state == "TURNING_180":
            if self.mag_heading_deg is None or (now - self.last_heading_time > 1.5):
                self.get_logger().warn("Dönüş sırasında pusula verisi kesildi! Motorlar durduruluyor.", throttle_duration_sec=2.0)
                self.stop_robot()
                return

            error = shortest_angular_error_deg(self.target_heading_deg, self.mag_heading_deg)

            if abs(error) <= self.heading_tolerance:
                self.heading_samples_in_tolerance += 1
                self.stop_robot()
                self.get_logger().info(
                    f"Açı tolerans içinde ({abs(error):.1f}° <= {self.heading_tolerance}°). "
                    f"Doğrulama Örneği: {self.heading_samples_in_tolerance}/{self.turn_required_samples}"
                )
                if self.heading_samples_in_tolerance >= self.turn_required_samples:
                    self.get_logger().info("✅ 180° Dönüş Tamamlandı! 3. Aşama: Dönüş sonrası sönümlenme bekleniyor...")
                    self.state = "RETURN_PAUSE"
                    self.pause_deadline = now + self.post_turn_pause
            else:
                self.heading_samples_in_tolerance = 0
                turn_cmd = String()
                # Pozitif hata: Sağa dön, Negatif hata: Sola dön
                turn_cmd.data = "MOTOR:RIGHT:80" if error > 0 else "MOTOR:LEFT:80"
                self.cmd_pub.publish(turn_cmd)
                self.get_logger().info(
                    f"Dönüş: Mevcut={self.mag_heading_deg:.1f}° | Hedef={self.target_heading_deg:.1f}° | Hata={error:.1f}°",
                    throttle_duration_sec=1.0
                )
            return

        # ---------------------------------------------------------
        # 3. AŞAMA: Dönüş Sonrası Sönümlenme Duraklaması
        # ---------------------------------------------------------
        if self.state == "RETURN_PAUSE":
            self.stop_robot()
            if now >= self.pause_deadline:
                self.get_logger().info("4. Aşama: Ters Rota Oynatılıyor...")
                self.state = "REPLAYING"
                self.replay_index = len(self.history) - 1
                self.replay_cmd_start = now
            return

        # ---------------------------------------------------------
        # 4. AŞAMA: Kaydedilen Rotaları Tersten Oynatma
        # ---------------------------------------------------------
        if self.state == "REPLAYING":
            if self.replay_index < 0:
                self.get_logger().info("🎉 ÜSSE GERİ DÖNÜŞ BAŞARIYLA TAMAMLANDI! Robot Ay Üssünde.")
                self.stop_robot()
                self.state = "COMPLETED"
                return

            orig_cmd, orig_duration = self.history[self.replay_index]
            elapsed = now - self.replay_cmd_start

            if elapsed < orig_duration:
                inverted_cmd_str = self.invert_command(orig_cmd)
                cmd_msg = String()
                cmd_msg.data = inverted_cmd_str
                self.cmd_pub.publish(cmd_msg)
                self.get_logger().info(
                    f"Geri Dönüş [{len(self.history) - self.replay_index}/{len(self.history)}]: "
                    f"Orijinal: {orig_cmd} -> Uygulanan: {inverted_cmd_str} | "
                    f"Kalan Süre: {(orig_duration - elapsed):.1f}s",
                    throttle_duration_sec=1.0
                )
            else:
                self.stop_robot()
                self.replay_index -= 1
                self.replay_cmd_start = now

    def stop_robot(self):
        cmd = String()
        cmd.data = "MOTOR:STOP"
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


