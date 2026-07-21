#!/usr/bin/env python3
"""
ArUco Etiket Algılama Düğümü (aruco_detector)
Earendil Bot - Raspberry Pi 5 (MRC 2026)
-----------------------------------------
1. Raspberry Pi 5 kamerası / USB Kamera üzerinden 5x5_100 ArUco etiketlerini algılar.
2. Trigonometri ve iğne deliği kamera modeli ile gerçek GERÇEK MESAFE (metre)
   ve AÇISAL SAPMA (derece) hesaplar.
3. Hakemler ve otonom navigasyon için etiket ID'sini (/aruco_id) okur.
4. Başsız (headless) çalışır (cv2.imshow kullanılmaz).
5. Seri portu meşgul etmez; ROS 2 konuları üzerinden haberleşir.

Yayınlanan ROS 2 Konuları (Topics):
  - /aruco_visible (std_msgs/Bool)     : Etiket görünüyor mu? (True/False)
  - /aruco_id      (std_msgs/Int32)    : Algılanan etiketin sayısal ID'si
  - /aruco_pose    (geometry_msgs/Point): x = Açısal sapma (derece), y = 0.0, z = Mesafe (metre)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32

import subprocess
import time
import cv2
import numpy as np
import math

TAG_SIZE_M = 0.20  # 20x20 cm etiket boyutu (MRC 2026 Şartnamesi)
HFOV_DEG = 62.2   # IMX219 kamera yatay görüş açısı


class RPI_Camera:
    """
    Raspberry Pi 5 kamera alıcısı. Önce rpicam-vid ile dener,
    olmazsa standart OpenCV VideoCapture (0) kullanır.
    """
    def __init__(self, width=1280, height=720, framerate=15):
        self.width = width
        self.height = height
        self.frame_size = width * height * 3 // 2
        self.process = None
        self.cap = None

        try:
            cmd = [
                'rpicam-vid',
                '-t', '0',
                '--width', str(width),
                '--height', str(height),
                '--framerate', str(framerate),
                '--codec', 'yuv420',
                '-n',
                '-o', '-'
            ]
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=self.frame_size * 2
            )
            time.sleep(0.5)
        except Exception:
            self.process = None
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        if self.process:
            try:
                raw = self.process.stdout.read(self.frame_size)
                if len(raw) != self.frame_size:
                    return False, None
                yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
                frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
                return True, frame
            except Exception:
                return False, None
        elif self.cap and self.cap.isOpened():
            return self.cap.read()
        return False, None

    def release(self):
        if self.process:
            self.process.terminate()
            self.process.wait()
        if self.cap:
            self.cap.release()


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ROS 2 Yayıncıları
        self.pose_pub = self.create_publisher(Point, '/aruco_pose', 10)
        self.visible_pub = self.create_publisher(Bool, '/aruco_visible', 10)
        self.id_pub = self.create_publisher(Int32, '/aruco_id', 10)

        self.cap = RPI_Camera()

        # ArUco Dictionary: 5X5_100 (MRC 2026 Şartnamesi)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        else:
            self.use_new_api = False

        # Kamera Odak Uzaklığı ve Odak Merkezi (Trigonometrik hesaplamalar için)
        self.IMAGE_W, self.IMAGE_H = 1280, 720
        self.fx = (self.IMAGE_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
        self.fy = self.fx
        self.cx = self.IMAGE_W / 2.0
        self.cy = self.IMAGE_H / 2.0

        self.get_logger().info("ArUco Tespit Düğümü Başlatıldı (Kütüphane: DICT_5X5_100, Headless Moda).")

        # 15 Hz döngü
        self.timer = self.create_timer(1.0 / 15.0, self.process_frame)

    def select_largest_marker(self, corners, ids):
        if corners is None or len(corners) == 0:
            return None, None

        largest_idx = max(
            range(len(corners)),
            key=lambda idx: cv2.contourArea(corners[idx][0].astype(np.float32))
        )
        return corners[largest_idx][0], ids[largest_idx][0]

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        visible_msg = Bool()
        visible_msg.data = False

        if ids is not None and len(ids) >= 1:
            marker_corners, marker_id = self.select_largest_marker(corners, ids)

            if marker_corners is not None:
                # Piksel alanı ve Kenar Uzunluğu
                area_px = cv2.contourArea(marker_corners.astype(np.float32))
                if area_px > 0:
                    side_px = math.sqrt(area_px)

                    # Trigonometrik Gerçek Mesafe (Metre cinsinden)
                    z_m = self.fx * TAG_SIZE_M / side_px

                    # Piksel Merkezi
                    c_x = int(np.mean(marker_corners[:, 0]))
                    x_cam = (c_x - self.cx) * z_m / self.fx

                    # Açısal Sapma (Derece cinsinden)
                    angle_x_deg = math.degrees(math.atan2(x_cam, z_m))

                    visible_msg.data = True

                    # 1. Mesafe ve Açı Yayınla
                    pose_msg = Point()
                    pose_msg.x = float(angle_x_deg)
                    pose_msg.y = 0.0
                    pose_msg.z = float(z_m)
                    self.pose_pub.publish(pose_msg)

                    # 2. Sayısal ArUco ID Yayınla
                    id_msg = Int32()
                    id_msg.data = int(marker_id)
                    self.id_pub.publish(id_msg)

                    self.get_logger().info(
                        f"ArUco Algılandı -> ID: {marker_id} | Mesafe: {z_m:.2f}m | Açı: {angle_x_deg:.1f}°",
                        throttle_duration_sec=1.0
                    )

        # Görünürlük Durumu Yayınla
        self.visible_pub.publish(visible_msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()