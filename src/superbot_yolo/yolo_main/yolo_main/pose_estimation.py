#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_msg.msg import PoseKeypoint, PoseResult
import numpy as np
from typing import List, Optional
from std_msgs.msg import Int32, String
from yolo_msg.msg import PersonInfo

import logging
import cv2
import time
import yaml

# TAMBAHAN: Import library AprilTag
from pupil_apriltags import Detector
from filterpy.kalman import KalmanFilter

class PoseEstimationNode(Node):

    def __init__(self) -> None:
        super().__init__("pose_estimation_node")
        logging.getLogger("ultralytics").setLevel(logging.CRITICAL)
        
        # Load YOLO model
        self.model = YOLO("/home/orin/superbot_ws/src/superbot_yolo/yolo_main/resource/train4.engine", task = "pose")
        # self.model = YOLO("/home/orin/superbot_ws/src/superbot_yolo/yolo_main/resource/train4.pt")
        
        # Inisialisasi Detektor AprilTag
        self.apriltag_detector = Detector(families='tag36h11',
                                                   nthreads=1,
                                                   quad_decimate=1.0,
                                                   quad_sigma=0.0,
                                                   refine_edges=1,
                                                   decode_sharpening=0.25,
                                                   debug=0)
        
        # kalman filter untuk tracking
        self.kalman_filter = None

        self.get_logger().info("Menginisialisasi kamera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Gagal membuka stream video dari kamera.")
            rclpy.shutdown()
            return

        # Atur resolusi 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # (Opsional) Verifikasi resolusi yang berhasil diatur
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Resolusi kamera diatur ke: {int(width)}x{int(height)}")
        # ---------------------------------------------

        # Publishers tetap dipertahankan untuk mengirim hasil deteksi ke node lain
        self.pose_publisher = self.create_publisher(PoseResult, "/yolo_pose_results", 1)
        self.image_publisher = self.create_publisher(Image, "/pose_estimation_result", 1)
        self.gesture_pub = self.create_publisher(String, "/yolo/gesture_detected", 10)
        self.distance_pub = self.create_publisher(PersonInfo, "/yolo/person_info", 10)

        self.bridge = CvBridge()
        
        # Variabel untuk State Machine Tracking AprilTag
        self.tracking_state = "SEARCHING"
        self.target_apriltag_id = 0
        self.tracked_person_bbox = None
        self.frames_since_lost = 0
        self.max_frames_lost = 15
        self.kalman_filter: Optional[KalmanFilter] = None

        # Konfigurasi untuk Skor Gabungan
        self.target_histogram = None
        self.w_iou = 0.7
        self.w_visual = 0.3
        self.apriltag_bonus_score = 100.0
        self.combined_score_threshold = 0.35

        # Class mapping
        self.class_names = {
            3: 'person', 0: 'batu', 1: 'gunting', 2: 'kertas'
        }

        # Variabel deteksi gestur
        self.gesture_detection_duration = 1.0
        self.gesture_history = {
            'batu': {'start_time': None, 'consecutive_detections': 0, 'last_sent': False},
            'gunting': {'start_time': None, 'consecutive_detections': 0, 'last_sent': False},
            'kertas': {'start_time': None, 'consecutive_detections': 0, 'last_sent': False}
        }
        self.min_consecutive_detections = 9
        
        self.minimum_iou_threshold = 0.5

        # Variabel untuk FPS
        self.prev_time = 0.0
        self.inference_time_ms = 0.0
        self.fps = 0.0

    def calculate_histogram(self, image, bbox):
        if bbox is None: return None
        x1, y1, x2, y2 = map(int, bbox)
        torso_y2 = y1 + int((y2 - y1) * 0.6)
        torso_roi = image[y1:torso_y2, x1:x2]
        if torso_roi.size == 0: return None
        hsv_roi = cv2.cvtColor(torso_roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv_roi], [0, 1], None, [50, 60], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        return hist

    def reset_gesture_history(self, exclude_gesture=None):
        for gesture in self.gesture_history:
            if gesture != exclude_gesture:
                self.gesture_history[gesture]['start_time'] = None
                self.gesture_history[gesture]['consecutive_detections'] = 0
                self.gesture_history[gesture]['last_sent'] = False

    def reset_tracking(self):
        self.get_logger().info("Tracking reset. Kembali ke mode SEARCHING.")
        self.tracking_state = "SEARCHING"
        self.tracked_person_bbox = None
        self.frames_since_lost = 0

    def calculate_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        if x2 <= x1 or y2 <= y1: return 0.0
        intersection = (x2 - x1) * (y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - intersection
        return intersection / union if union > 0 else 0.0

    def init_kalman_filter(self, initial_bbox):
        self.kalman_filter = KalmanFilter(dim_x=8, dim_z=4)
        self.kalman_filter.F = np.array([[1,0,0,0,1,0,0,0], [0,1,0,0,0,1,0,0], [0,0,1,0,0,0,1,0], [0,0,0,1,0,0,0,1], [0,0,0,0,1,0,0,0], [0,0,0,0,0,1,0,0], [0,0,0,0,0,0,1,0], [0,0,0,0,0,0,0,1]], dtype=float)
        self.kalman_filter.H = np.array([[1,0,0,0,0,0,0,0], [0,1,0,0,0,0,0,0], [0,0,1,0,0,0,0,0], [0,0,0,1,0,0,0,0]], dtype=float)
        self.kalman_filter.R[2:,2:] *= 10.
        self.kalman_filter.P[4:,4:] *= 1000. 
        self.kalman_filter.P *= 10.
        self.kalman_filter.Q[-1,-1] *= 0.01
        self.kalman_filter.Q[4:,4:] *= 0.01
        self.kalman_filter.x[:4] = self.bbox_to_zx(initial_bbox)

    def bbox_to_zx(self, bbox):
        w = bbox[2] - bbox[0]; h = bbox[3] - bbox[1]
        x = bbox[0] + w/2.; y = bbox[1] + h/2.
        s = w * h; r = w / float(h) if h != 0 else 0
        return np.array([x, y, s, r]).reshape((4, 1))

    def zx_to_bbox(self, zx):
        """Mengubah state [cx,cy,s,r] kembali ke bbox [x1,y1,x2,y2]"""
        if zx[2, 0] * zx[3, 0] <= 0 or zx[2, 0] <= 0:
            return None 
        
        w = np.sqrt(zx[2, 0] * zx[3, 0])
        h = zx[2, 0] / w
        
        if w <= 0 or h <= 0:
            return None

        x1 = zx[0, 0] - w/2.
        y1 = zx[1, 0] - h/2.
        return np.array([x1, y1, x1+w, y1+h]).flatten()

    def is_box_inside(self, box_inner, box_outer):
        ix1, iy1, ix2, iy2 = box_inner
        ox1, oy1, ox2, oy2 = box_outer
        return ox1 <= ix1 and oy1 <= iy1 and ox2 >= ix2 and oy2 >= iy2

    def validate_gesture_with_tracked_person(self, detected_gestures, tracked_person_box):
        if tracked_person_box is None: return []
        valid_gestures = [g for g in detected_gestures if self.is_box_inside(g['box'], tracked_person_box)]
        return valid_gestures

    def process_hand_gesture(self, detected_gestures, tracked_person_box):
        current_time = time.time()
        valid_gestures = self.validate_gesture_with_tracked_person(detected_gestures, tracked_person_box)
        
        if len(valid_gestures) >= 2 or not valid_gestures:
            self.reset_gesture_history()
            return
        
        best_gesture = max(valid_gestures, key=lambda x: x['confidence'])
        gesture_name = best_gesture['name']
        self.reset_gesture_history(exclude_gesture=gesture_name)

        if gesture_name not in self.gesture_history: return

        gesture_data = self.gesture_history[gesture_name]
        if gesture_data['start_time'] is None:
            gesture_data['start_time'] = current_time
            gesture_data['consecutive_detections'] = 1
            gesture_data['last_sent'] = False
        else:
            gesture_data['consecutive_detections'] += 1
            time_elapsed = current_time - gesture_data['start_time']
            if (time_elapsed >= self.gesture_detection_duration and 
                gesture_data['consecutive_detections'] >= self.min_consecutive_detections and 
                not gesture_data['last_sent']):
                self.send_gesture_command(gesture_name)
                gesture_data['last_sent'] = True
                if gesture_name == 'kertas': self.reset_tracking()
    
    def send_gesture_command(self, gesture_name):
        msg = String()
        command_map = {'batu': 'STOP', 'kertas': 'FOLLOW_MODE', 'gunting': 'GO'}
        if gesture_name in command_map:
            msg.data = command_map[gesture_name]
            self.gesture_pub.publish(msg)
            self.get_logger().info(f"Published gesture command: {msg.data}")

    # DIUBAH: Fungsi callback menjadi loop utama `run_loop`
    def run_loop(self) -> None:
        while rclpy.ok():
            current_time = time.time()
            if self.prev_time > 0: self.fps = 1.0 / (current_time - self.prev_time)
            self.prev_time = current_time

            # DIUBAH: Baca frame langsung dari kamera
            ret, img = self.cap.read()
            if not ret:
                self.get_logger().info("Frame tidak dapat dibaca, loop berhenti.")
                break

            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            yolo_results = self.model(img, verbose=False)
            apriltag_detections = self.apriltag_detector.detect(gray_img)
            self.inference_time_ms = yolo_results[0].speed['inference']

            pose_result = PoseResult()
            pose_result.header.stamp = self.get_clock().now().to_msg()
            pose_result.header.frame_id = "pose_estimation"

            if yolo_results and yolo_results[0].boxes is not None:
                boxes = yolo_results[0].boxes
                person_detections = [{'box': box.cpu().numpy(), 'confidence': float(conf.item())} for box, cls, conf in zip(boxes.xyxy, boxes.cls, boxes.conf) if self.class_names.get(int(cls.item())) == 'person']
                detected_gestures = [{'name': self.class_names.get(int(cls.item())), 'confidence': float(conf.item()), 'box': box.cpu().numpy()} for box, cls, conf in zip(boxes.xyxy, boxes.cls, boxes.conf) if self.class_names.get(int(cls.item())) in ['batu', 'gunting', 'kertas']]

                # --- LOGIKA UTAMA TRACKING ---
                if self.tracking_state == "SEARCHING":
                    target_tag = next((tag for tag in apriltag_detections if tag.tag_id == self.target_apriltag_id), None)
                    if target_tag and person_detections:
                        tag_center = target_tag.center
                        closest_person_box = None; min_dist = float('inf')
                        for person in person_detections:
                            p_box = person['box']
                            if self.is_box_inside([tag_center[0]-5, tag_center[1]-5, tag_center[0]+5, tag_center[1]+5], p_box):
                                person_center_x, person_center_y = (p_box[0] + p_box[2]) / 2, (p_box[1] + p_box[3]) / 2
                                dist = np.linalg.norm([tag_center[0] - person_center_x, tag_center[1] - person_center_y])
                                if dist < min_dist: min_dist, closest_person_box = dist, p_box
                        
                        if closest_person_box is not None:
                            self.tracked_person_bbox = closest_person_box
                            self.init_kalman_filter(self.tracked_person_bbox)
                            self.target_histogram = self.calculate_histogram(img, self.tracked_person_bbox)
                            if self.target_histogram is not None:
                                self.tracking_state = "TRACKING"; self.frames_since_lost = 0
                                self.get_logger().info(f"Target Acquired (ID: {self.target_apriltag_id}). Switching to TRACKING mode.")
                            else: self.reset_tracking()
                
                elif self.tracking_state == "TRACKING":
                    if self.kalman_filter is None or self.target_histogram is None: self.reset_tracking()
                    else:
                        self.kalman_filter.predict(); predicted_bbox = self.zx_to_bbox(self.kalman_filter.x)
                        candidates = []
                        if person_detections and predicted_bbox is not None:
                            for person in person_detections:
                                p_box = person['box']
                                iou_score = self.calculate_iou(predicted_bbox, p_box)
                                current_hist = self.calculate_histogram(img, p_box)
                                visual_score = cv2.compareHist(self.target_histogram, current_hist, cv2.HISTCMP_CORREL) if current_hist is not None else 0.0
                                combined_score = (self.w_iou * iou_score) + (self.w_visual * visual_score)
                                for tag in apriltag_detections:
                                    if tag.tag_id == self.target_apriltag_id and self.is_box_inside([tag.center[0]-5, tag.center[1]-5, tag.center[0]+5, tag.center[1]+5], p_box):
                                        combined_score += self.apriltag_bonus_score; break
                                candidates.append({'box': p_box, 'score': combined_score})
                        
                        best_match = None
                        if candidates:
                            best_candidate = max(candidates, key=lambda x: x['score'])
                            if best_candidate['score'] > self.combined_score_threshold: best_match = best_candidate['box']

                        if best_match is not None: self.kalman_filter.update(self.bbox_to_zx(best_match)); self.frames_since_lost = 0
                        else: self.frames_since_lost += 1
                        
                        self.tracked_person_bbox = self.zx_to_bbox(self.kalman_filter.x)
                        if self.tracked_person_bbox is None:
                            self.frames_since_lost += 1
                        if self.frames_since_lost > self.max_frames_lost: self.reset_tracking()

                person_to_validate_against = self.tracked_person_bbox
                self.process_hand_gesture(detected_gestures, person_to_validate_against)

                if (hasattr(yolo_results[0], "keypoints") and yolo_results[0].keypoints is not None and person_to_validate_against is not None):
                    for i, box in enumerate(yolo_results[0].boxes):
                        if self.class_names.get(int(box.cls)) == 'person' and self.calculate_iou(box.xyxy[0].cpu().numpy(), person_to_validate_against) > 0.8:
                            kp_xy = yolo_results[0].keypoints[i].xy[0].cpu().numpy()
                            kp_conf = yolo_results[0].keypoints[i].conf[0].cpu().numpy()
                            person_keypoints = [PoseKeypoint(id=idx, x=float(k[0]), y=float(k[1]), confidence=float(c)) for idx, (k, c) in enumerate(zip(kp_xy, kp_conf))]
                            pose_result.keypoints.extend(person_keypoints)
                            height_p = self.calculate_head_neck(person_keypoints)
                            if height_p > 0:
                                distance = int((25 * 562) / height_p)
                                (text_width, _), _ = cv2.getTextSize(f"Jarak: {distance} cm", cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                                img_height, img_width, _ = img.shape

                                cv2.putText(img, f"Jarak: {distance} cm", (img_width - text_width - 10, img_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                                person_info_msg = PersonInfo(distance_cm=distance, center_x_obj=int(person_keypoints[0].x))
                                self.distance_pub.publish(person_info_msg)
                            break
                
                # Gambar-gambar anotasi
                if self.tracked_person_bbox is not None:
                    x1, y1, x2, y2 = map(int, self.tracked_person_bbox)
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img, f"TARGET (ID:{self.target_apriltag_id})", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                self.pose_publisher.publish(pose_result)
                self.draw_gesture_status(img, detected_gestures, person_to_validate_against)
                self.draw_detected_gestures(img, detected_gestures, person_to_validate_against)
                self.draw_tracking_status(img)
                
                h, w, _ = img.shape
                cv2.putText(img, f"FPS: {self.fps:.1f}", (w - 150, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(img, f"Inference: {self.inference_time_ms:.1f} ms", (w - 320, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Publikasikan gambar hasil anotasi ke topik ROS
                img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                self.image_publisher.publish(img_msg)

            cv2.imshow("Direct Camera Capture - yolo", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Tombol 'q' ditekan, mematikan node.")
                break
        
        self.get_logger().info("Menutup kamera dan jendela.")
        self.cap.release()
        cv2.destroyAllWindows()

    def calculate_head_neck(self, keypoints: List[PoseKeypoint]) -> float:
        try:
            head_coor = np.array([keypoints[0].x, keypoints[0].y])
            neck_coor = np.array([keypoints[1].x, keypoints[1].y])
            if np.array_equal(head_coor, [0, 0]) or np.array_equal(neck_coor, [0, 0]): return -1.0
            return np.linalg.norm(head_coor - neck_coor)
        except IndexError: return -1.0
    
    def draw_gesture_status(self, img, detected_gestures, tracked_person_box):
        y_offset = 30
        status_text = "TARGET Locked" if tracked_person_box is not None else "TARGET Not Found"
        color = (0, 255, 0) if tracked_person_box is not None else (0, 0, 255)
        cv2.putText(img, status_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2); y_offset += 25
        valid_gestures = self.validate_gesture_with_tracked_person(detected_gestures, tracked_person_box)
        if len(valid_gestures) >= 2:
            cv2.putText(img, f"Multiple gestures detected ({len(valid_gestures)})", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2); y_offset += 25
        for gesture_name, data in self.gesture_history.items():
            if data['start_time'] is not None:
                remaining_time = max(0, self.gesture_detection_duration - (time.time() - data['start_time']))
                status_text = f"{gesture_name}: {remaining_time:.1f}s ({data['consecutive_detections']})"
                color = (0, 255, 0) if data['last_sent'] else (0, 165, 255)
                cv2.putText(img, status_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2); y_offset += 25

    def draw_detected_gestures(self, img, detected_gestures, tracked_person_box):
        valid_gestures = self.validate_gesture_with_tracked_person(detected_gestures, tracked_person_box)
        for gesture in detected_gestures:
            x1, y1, x2, y2 = map(int, gesture['box'])
            is_valid = gesture in valid_gestures
            color = (128, 128, 128) if not is_valid else ((0, 0, 255) if len(valid_gestures) >= 2 else (0, 255, 0))
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, f"{gesture['name']}: {gesture['confidence']:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def draw_tracking_status(self, img):
        h, w, _ = img.shape
        status_text = f"Mode: {self.tracking_state}"
        color = (0, 255, 0) if self.tracking_state == "TRACKING" else (0, 165, 255)
        cv2.putText(img, status_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

def main(args=None) -> None:
    rclpy.init(args=args)
    pose_estimation_node = PoseEstimationNode()
    
    try:
        pose_estimation_node.run_loop()
    except KeyboardInterrupt:
        pass 
    finally:
        pose_estimation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()