#!/usr/bin/env python3

import glob
import os
import re
import threading

import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


CLASS_NAMES = ["number%d" % i for i in range(10)]
TEMPLATE_SIZE = (64, 64)


class Detection(object):
    def __init__(self, class_name, center, bbox, score):
        self.class_name = class_name
        self.center = center
        self.bbox = bbox
        self.score = score


class OpenCvNumberDetector(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("yolo")
        self.workspace_path = os.path.abspath(os.path.join(self.package_path, "..", ".."))
        self.resources_path = rospy.get_param(
            "~resources_path", os.path.join(self.workspace_path, "Resources")
        )

        self.image_topic = rospy.get_param("~image_topic", "/gripper_camera/image_raw")
        self.info_topic = rospy.get_param("~info_topic", "/yolo_info")
        self.result_topic = rospy.get_param("~result_topic", "/yolo_result")
        self.result_img_topic = rospy.get_param("~result_img_topic", "/yolo_result_img")
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))

        self.block_min_saturation = int(rospy.get_param("~block_min_saturation", 70))
        self.block_min_value = int(rospy.get_param("~block_min_value", 60))
        self.black_max_channel = int(
            rospy.get_param("~black_max_channel", rospy.get_param("~black_max_value", 95))
        )
        self.min_block_area_ratio = float(
            rospy.get_param("~min_block_area_ratio", rospy.get_param("~min_white_area_ratio", 0.00025))
        )
        self.max_block_area_ratio = float(
            rospy.get_param("~max_block_area_ratio", rospy.get_param("~max_white_area_ratio", 0.25))
        )
        self.min_digit_area_ratio = float(rospy.get_param("~min_digit_area_ratio", 0.004))
        self.max_digit_area_ratio = float(rospy.get_param("~max_digit_area_ratio", 0.35))
        self.match_threshold = float(rospy.get_param("~match_threshold", 0.58))
        self.debug_log = bool(rospy.get_param("~debug_log", False))

        self.bridge = CvBridge()
        self.templates = self._load_templates()
        self.lock = threading.Lock()
        self.current_classes = []
        self.current_centers = []
        self.current_result_img = Image()
        self.all_classes_name = self._build_info_message()

        self.info_pub = rospy.Publisher(self.info_topic, String, queue_size=10)
        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10)
        self.result_img_pub = rospy.Publisher(self.result_img_topic, Image, queue_size=10)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )

    def _build_info_message(self):
        msg = String()
        msg.data = "all classes name:" + "".join("%s," % name for name in CLASS_NAMES)
        return msg

    def _load_templates(self):
        templates = []
        pattern = os.path.join(
            self.resources_path, "models", "number*_01", "materials", "textures", "number*.png"
        )
        for path in sorted(glob.glob(pattern)):
            match = re.search(r"number(\d)\.png$", os.path.basename(path))
            if not match:
                continue

            class_name = "number%s" % match.group(1)
            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if img is None:
                rospy.logwarn("OpenCV detector could not read template: %s", path)
                continue

            bgr, alpha = self._to_bgr_and_alpha(img)
            for mask in self._extract_face_templates(bgr, alpha):
                templates.append((class_name, mask))

        if not templates:
            rospy.logwarn(
                "OpenCV detector found no texture templates under %s; using font fallback.",
                self.resources_path,
            )
            templates = self._build_font_templates()

        rospy.loginfo("OpenCV detector loaded %d number templates.", len(templates))
        return templates

    def _to_bgr_and_alpha(self, img):
        if len(img.shape) == 2:
            alpha = np.full(img.shape, 255, dtype=np.uint8)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), alpha
        if len(img.shape) == 3 and img.shape[2] == 4:
            return img[:, :, :3], img[:, :, 3]
        alpha = np.full(img.shape[:2], 255, dtype=np.uint8)
        return img[:, :, :3], alpha

    def _extract_face_templates(self, bgr, alpha):
        face_boxes = self._find_template_face_boxes(bgr, alpha)
        if not face_boxes:
            return self._extract_component_templates(bgr, alpha)

        templates = []
        for x, y, bw, bh in face_boxes:
            margin = max(3, int(min(bw, bh) * 0.06))
            roi = bgr[y + margin : y + bh - margin, x + margin : x + bw - margin]
            roi_alpha = alpha[y + margin : y + bh - margin, x + margin : x + bw - margin]
            if roi.size == 0:
                continue
            dark = self._make_dark_mask(roi, roi_alpha)
            dark = self._keep_useful_dark_components(dark, 0.001, 0.5)
            normalized = self._normalize_mask(dark)
            if normalized is not None:
                templates.append(normalized)
        return templates

    def _find_template_face_boxes(self, bgr, alpha):
        h, w = bgr.shape[:2]
        face_boxes = []

        if alpha is not None and np.any(alpha == 0):
            alpha_mask = np.zeros_like(alpha, dtype=np.uint8)
            alpha_mask[alpha > 0] = 255
            alpha_mask = cv2.morphologyEx(
                alpha_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            )
            count, labels, stats, _ = cv2.connectedComponentsWithStats(alpha_mask, 8)
            for index in range(1, count):
                x, y, bw, bh, area = stats[index]
                aspect = float(bw) / float(max(1, bh))
                if area > h * w * 0.02 and bw > w * 0.2 and bh > h * 0.2 and 0.75 < aspect < 1.3:
                    face_boxes.append((x, y, bw, bh))

        border_mask = self._make_dark_mask(bgr, alpha) * 255
        contours = self._find_external_contours(border_mask)

        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            aspect = float(bw) / float(max(1, bh))
            if bw > w * 0.2 and bh > h * 0.2 and 0.75 < aspect < 1.3:
                face_boxes.append((x, y, bw, bh))

        return self._merge_boxes(face_boxes)

    def _extract_component_templates(self, bgr, alpha):
        dark = self._make_dark_mask(bgr, alpha)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
        templates = []
        image_area = float(bgr.shape[0] * bgr.shape[1])
        for index in range(1, count):
            x, y, w, h, area = stats[index]
            fill = area / float(max(1, w * h))
            if area < image_area * 0.001 or fill < 0.08 or w < 20 or h < 20:
                continue
            component = np.zeros_like(dark)
            component[labels == index] = 1
            normalized = self._normalize_mask(component)
            if normalized is not None:
                templates.append(normalized)
        return templates

    def _merge_boxes(self, boxes):
        merged = []
        for box in sorted(boxes, key=lambda item: item[2] * item[3], reverse=True):
            duplicate = False
            for kept in merged:
                if self._bbox_iou(box, kept) > 0.6:
                    duplicate = True
                    break

                x, y, w, h = box
                kx, ky, kw, kh = kept
                cx = x + w / 2.0
                cy = y + h / 2.0
                kcx = kx + kw / 2.0
                kcy = ky + kh / 2.0
                center_dist = ((cx - kcx) ** 2 + (cy - kcy) ** 2) ** 0.5
                size = max(w, h, kw, kh)
                if center_dist < 0.05 * size and abs(w - kw) < 0.1 * size and abs(h - kh) < 0.1 * size:
                    duplicate = True
                    break

            if not duplicate:
                merged.append(box)
        return sorted(merged, key=lambda item: (item[1], item[0]))

    def _build_font_templates(self):
        templates = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        for digit in range(10):
            for angle in (0, 90, 180, 270):
                canvas = np.zeros((96, 96), dtype=np.uint8)
                text = str(digit)
                scale = 2.7
                thickness = 8
                (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
                org = ((96 - tw) // 2, (96 + th) // 2 - baseline)
                cv2.putText(canvas, text, org, font, scale, 1, thickness, cv2.LINE_AA)
                if digit in (6, 9):
                    y = min(88, org[1] + 10)
                    cv2.line(canvas, (26, y), (70, y), 1, 6)
                rotated = self._rotate_mask(canvas, angle)
                normalized = self._normalize_mask(rotated)
                if normalized is not None:
                    templates.append(("number%d" % digit, normalized))
        return templates

    def _rotate_mask(self, mask, angle):
        if angle == 90:
            return cv2.rotate(mask, cv2.ROTATE_90_CLOCKWISE)
        if angle == 180:
            return cv2.rotate(mask, cv2.ROTATE_180)
        if angle == 270:
            return cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return mask

    def image_callback(self, ros_img):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("OpenCV detector image conversion failed: %s", exc)
            return

        detections, debug_img = self.detect(frame)
        result_img = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")

        with self.lock:
            self.current_classes = [det.class_name for det in detections]
            self.current_centers = [det.center for det in detections]
            self.current_result_img = result_img

    def detect(self, frame):
        debug_img = frame.copy()
        h, w = frame.shape[:2]
        image_area = float(h * w)
        block_mask = self._make_block_mask(frame)
        contours = self._find_external_contours(block_mask)

        detections = []
        min_area = max(80.0, image_area * self.min_block_area_ratio)
        max_area = image_area * self.max_block_area_ratio

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 12 or bh < 12:
                continue

            aspect = float(bw) / float(max(1, bh))
            fill = area / float(max(1, bw * bh))
            if aspect < 0.35 or aspect > 2.8 or fill < 0.25:
                continue

            roi = frame[y : y + bh, x : x + bw]
            if roi.size == 0:
                continue

            dark = self._make_dark_mask(roi)
            dark = self._remove_roi_border(dark, 0.04)
            dark = self._keep_useful_dark_components(
                dark, self.min_digit_area_ratio, self.max_digit_area_ratio
            )
            normalized = self._normalize_mask(dark)
            if normalized is None:
                continue

            class_name, score = self._classify(normalized)
            if class_name is None or score < self.match_threshold:
                if self.debug_log:
                    rospy.loginfo("Rejected candidate score %.3f at [%d,%d,%d,%d]", score, x, y, bw, bh)
                continue

            center = (int(x + bw / 2), int(y + bh / 2))
            detections.append(Detection(class_name, center, (x, y, bw, bh), score))

        detections = self._merge_detections(detections)
        self._draw_detections(debug_img, detections)
        return detections, debug_img

    def _make_block_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        mask[(saturation > self.block_min_saturation) & (value > self.block_min_value)] = 255
        mask = cv2.medianBlur(mask, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def _make_dark_mask(self, frame, alpha=None):
        if len(frame.shape) == 2:
            max_channel = frame
        else:
            max_channel = np.max(frame[:, :, :3], axis=2)

        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        mask[max_channel < self.black_max_channel] = 1
        if alpha is not None:
            mask[alpha == 0] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        return mask

    def _remove_roi_border(self, mask, margin_ratio):
        cleaned = mask.copy()
        h, w = cleaned.shape[:2]
        margin = max(2, int(min(h, w) * margin_ratio))
        cleaned[:margin, :] = 0
        cleaned[h - margin :, :] = 0
        cleaned[:, :margin] = 0
        cleaned[:, w - margin :] = 0
        return cleaned

    def _keep_useful_dark_components(self, mask, min_area_ratio, max_area_ratio):
        mask = (mask > 0).astype(np.uint8)
        total_area = float(mask.shape[0] * mask.shape[1])
        min_area = max(6.0, total_area * min_area_ratio)
        max_area = max(min_area + 1.0, total_area * max_area_ratio)

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        kept = np.zeros_like(mask)
        for index in range(1, count):
            x, y, w, h, area = stats[index]
            if area < min_area or area > max_area:
                continue
            aspect = float(w) / float(max(1, h))
            if aspect > 12.0 or aspect < 0.08:
                continue
            kept[labels == index] = 1
        return kept

    def _normalize_mask(self, mask):
        mask = (mask > 0).astype(np.uint8)
        points = cv2.findNonZero(mask)
        if points is None:
            return None

        x, y, w, h = cv2.boundingRect(points)
        if w < 4 or h < 4:
            return None

        crop = mask[y : y + h, x : x + w]
        target_w, target_h = TEMPLATE_SIZE
        pad = 4
        scale = min(
            float(target_w - 2 * pad) / float(max(1, w)),
            float(target_h - 2 * pad) / float(max(1, h)),
        )
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

        canvas = np.zeros((target_h, target_w), dtype=np.uint8)
        x0 = (target_w - new_w) // 2
        y0 = (target_h - new_h) // 2
        canvas[y0 : y0 + new_h, x0 : x0 + new_w] = resized
        return canvas

    def _classify(self, normalized):
        best_name = None
        best_score = 0.0
        for class_name, template in self.templates:
            score = self._mask_score(normalized, template)
            if score > best_score:
                best_name = class_name
                best_score = score
        return best_name, best_score

    def _mask_score(self, observed, template):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        observed_bool = observed > 0
        template_bool = template > 0
        observed_dilated = cv2.dilate(observed, kernel, iterations=1) > 0
        template_dilated = cv2.dilate(template, kernel, iterations=1) > 0

        template_coverage = np.logical_and(observed_dilated, template_bool).sum() / float(
            max(1, template_bool.sum())
        )
        observed_coverage = np.logical_and(template_dilated, observed_bool).sum() / float(
            max(1, observed_bool.sum())
        )
        area_balance = min(observed_bool.sum(), template_bool.sum()) / float(
            max(1, max(observed_bool.sum(), template_bool.sum()))
        )
        return 0.45 * template_coverage + 0.45 * observed_coverage + 0.10 * area_balance

    def _merge_detections(self, detections):
        merged = []
        for det in sorted(detections, key=lambda item: item.score, reverse=True):
            duplicate = False
            for kept in merged:
                if self._bbox_iou(det.bbox, kept.bbox) > 0.25:
                    duplicate = True
                    break
                dx = det.center[0] - kept.center[0]
                dy = det.center[1] - kept.center[1]
                dist = (dx * dx + dy * dy) ** 0.5
                scale = max(det.bbox[2], det.bbox[3], kept.bbox[2], kept.bbox[3])
                if dist < 0.35 * scale:
                    duplicate = True
                    break
            if not duplicate:
                merged.append(det)
        return merged

    def _bbox_iou(self, a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        x1 = max(ax, bx)
        y1 = max(ay, by)
        x2 = min(ax + aw, bx + bw)
        y2 = min(ay + ah, by + bh)
        inter_w = max(0, x2 - x1)
        inter_h = max(0, y2 - y1)
        intersection = inter_w * inter_h
        union = aw * ah + bw * bh - intersection
        return float(intersection) / float(max(1, union))

    def _find_external_contours(self, mask):
        found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(found) == 2:
            contours, _ = found
        else:
            _, contours, _ = found
        return contours

    def _draw_detections(self, image, detections):
        for det in detections:
            x, y, w, h = det.bbox
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
            cv2.circle(image, det.center, 4, (0, 0, 255), -1)
            label = "%s %.2f" % (det.class_name, det.score)
            text_y = y - 8 if y > 20 else y + h + 22
            cv2.putText(image, label, (x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    def publish_loop(self):
        rate = rospy.Rate(self.publish_rate)
        count = 0
        while not rospy.is_shutdown():
            with self.lock:
                classes = list(self.current_classes)
                centers = list(self.current_centers)
                result_img = self.current_result_img

            result = String()
            result.data = "classes:"
            for class_name in classes:
                result.data += "%s," % class_name
            result.data += "center:"
            for center in centers:
                result.data += "[%d,%d]," % (int(center[0]), int(center[1]))

            self.result_pub.publish(result)
            if count % max(1, int(self.publish_rate)) == 0:
                self.info_pub.publish(self.all_classes_name)
            self.result_img_pub.publish(result_img)

            count += 1
            rate.sleep()


def main():
    rospy.init_node("yolo_detect")
    detector = OpenCvNumberDetector()
    publish_thread = threading.Thread(target=detector.publish_loop)
    publish_thread.daemon = True
    publish_thread.start()
    rospy.spin()


if __name__ == "__main__":
    main()
