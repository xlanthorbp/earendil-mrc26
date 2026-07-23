#!/usr/bin/env python3
"""
Masaüstü/PC Kamera ArUco Algılayıcı & 1902A LCD Seri Gönderici Test Betiği
-------------------------------------------------------------------------
1. Bilgisayarın web kamerasını (OpenCV VideoCapture) açar.
2. DICT_5X5_100 ArUco etiketlerini (18x18 cm) gerçek zamanlı olarak algılar.
3. Mesafe 35 cm'den YAKINSA (<= 35 cm) ID'yi Arduino'ya Seri Port üzerinden iletir:
     "ID:<marker_id>\n"
4. Masaüstü/PC web kameraları için mesafe kalibrasyon desteği içerir.

Kullanım:
    python3 standalone_aruco_lcd.py --port /dev/ttyACM0 --camera 0
"""

import argparse
import sys
import time
import math
import threading
import cv2
import numpy as np

try:
    import serial
except ImportError:
    print("[UYARI] 'pyserial' kütüphanesi yüklü değil. Seri port kapalı çalışacak.")
    print("Yüklemek için: pip install pyserial")
    serial = None

HFOV_DEG = 78.0  # PC kameraları için geniş açı varsayılanı


class SerialWriterThread(threading.Thread):
    """
    Kamera akışının (FPS) seri port yazma işleminden etkilenmemesi için
    seri port mesajlarını arka planda gönderen thread.
    """
    def __init__(self, ser_instance):
        super().__init__(daemon=True)
        self.ser = ser_instance
        self.latest_msg = "NONE\n"
        self.last_sent_msg = None
        self.running = True

    def set_message(self, msg: str):
        self.latest_msg = msg

    def run(self):
        while self.running:
            msg = self.latest_msg
            if msg != self.last_sent_msg:
                self.last_sent_msg = msg
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(msg.encode('utf-8'))
                        if msg.startswith("ID:"):
                            print(f"[SERİ GÖNDERİLDİ] {msg.strip()}")
                        else:
                            print("[SERİ GÖNDERİLDİ] NONE (Mesafe eşik dışında veya etiket yok)")
                    except Exception as err:
                        print(f"[SERİ HATA] {err}")
            time.sleep(0.1)  # 10 Hz güncelleme hızı

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"NONE\n")
            except Exception:
                pass


def select_largest_marker(corners, ids):
    if corners is None or len(corners) == 0:
        return None, None
    largest_idx = max(
        range(len(corners)),
        key=lambda idx: cv2.contourArea(corners[idx][0].astype(np.float32))
    )
    return corners[largest_idx][0], ids[largest_idx][0]


def main():
    parser = argparse.ArgumentParser(description="PC Kamera ArUco & 1902A LCD Yüksek FPS Testi")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Arduino Seri Portu (Örn: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Seri Port Baudrate (Varsayılan: 115200)")
    parser.add_argument("--camera", type=int, default=0, help="Kamera İndeksi (Varsayılan: 0)")
    parser.add_argument("--tag-size", type=float, default=0.18, help="Etiket Boyutu Metre Cinsinden (Varsayılan: 0.18m -> 18x18 cm)")
    parser.add_argument("--max-dist", type=float, default=35.0, help="Ekrana yazdırmak için maks mesafe eşiği cm cinsinden (Varsayılan: 35 cm)")
    parser.add_argument("--fov", type=float, default=78.0, help="PC Web Kamerası Görüş Açısı / FOV (Varsayılan: 78.0 derece)")
    parser.add_argument("--calib-scale", type=float, default=1.0, help="Mesafe Düzeltme Çarpanı (Varsayılan: 1.0)")
    args = parser.parse_args()

    # 1. Seri Port Bağlantısı
    ser = None
    if serial:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
            print(f"[BİLGİ] Arduino seri portu açıldı: {args.port} ({args.baud} baud)")
            time.sleep(2.0)  # Arduino reset beklemesi
        except Exception as e:
            print(f"[UYARI] Seri port açılamadı ({args.port}): {e}")
            print("[BİLGİ] Yalnızca kamera önizleme modunda çalışılıyor.")

    # Seri Gönderici Thread'ini Başlat
    serial_thread = SerialWriterThread(ser)
    serial_thread.start()

    # 2. Kamera Bağlantısı & 640x480 MJPEG Ayarı (FPS Artışı İçin)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[HATA] Kamera açılamadı (İndeks: {args.camera})")
        sys.exit(1)

    # 640x480 Çözünürlük ve MJPEG Format Zorlaması
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"[BİLGİ] Kamera çözünürlüğü ayarlandı: {int(actual_w)}x{int(actual_h)}")

    # 3. ArUco Kütüphanesi Tanımlaması (5X5_100)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        detector = cv2.aruco.ArucoDetector(aruco_dict)
        use_new_api = True
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        use_new_api = False

    print("\n-----------------------------------------------------")
    print("Yüksek FPS ArUco LCD Testi Başlatıldı.")
    print(f" - Çözünürlük     : {int(actual_w)}x{int(actual_h)}")
    print(f" - Etiket Boyutu  : {args.tag_size * 100:.0f}x{args.tag_size * 100:.0f} cm")
    print(f" - Mesafe Eşiği   : {args.max_dist:.1f} cm")
    print(f" - Arka Plan Seri : Aktif (Non-blocking Thread)")
    print(" - Çıkış için kamera penceresindeyken 'q' tuşuna basın.")
    print("-----------------------------------------------------\n")

    prev_frame_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[HATA] Kameradan kare alınamadı.")
                break

            # Canlı FPS Hesaplama
            new_frame_time = time.time()
            fps = 1.0 / (new_frame_time - prev_frame_time + 1e-6)
            prev_frame_time = new_frame_time

            h, w = frame.shape[:2]
            fx = (w / 2.0) / math.tan(math.radians(args.fov / 2.0))

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = np.ascontiguousarray(gray)

            # Marker Algılama
            if use_new_api:
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)

            target_id = None
            dist_cm = None

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                marker_corners, marker_id = select_largest_marker(corners, ids)

                if marker_corners is not None:
                    area_px = cv2.contourArea(marker_corners.astype(np.float32))
                    if area_px > 0:
                        side_px = math.sqrt(area_px)
                        raw_z_m = (fx * args.tag_size) / side_px
                        dist_cm = (raw_z_m * 100.0) * args.calib_scale
                        target_id = int(marker_id)

                        color = (0, 255, 0) if dist_cm <= args.max_dist else (0, 165, 255)
                        status_text = "YAKIN (LCD AKTIF)" if dist_cm <= args.max_dist else "UZAK (LCD BOS)"
                        
                        cv2.putText(
                            frame,
                            f"ID: {target_id} | Mesafe: {dist_cm:.1f} cm [{status_text}]",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color,
                            2
                        )

            else:
                cv2.putText(
                    frame,
                    "ArUco: Bulunamadi",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            # Ekrane Canlı FPS Yazdır
            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (w - 120, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

            # Eşik Kontrolü: 35 cm veya daha yakınsa ID'yi arka plan thread'ine ilet
            if target_id is not None and dist_cm is not None and dist_cm <= args.max_dist:
                serial_thread.set_message(f"ID:{target_id}\n")
            else:
                serial_thread.set_message("NONE\n")

            # Kamera Penceresini Göster
            cv2.imshow("ArUco 1902A LCD Testi (Yuksek FPS)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    finally:
        serial_thread.stop()
        cap.release()
        cv2.destroyAllWindows()
        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
        print("[BİLGİ] Test betiği sonlandırıldı.")


if __name__ == "__main__":
    main()
