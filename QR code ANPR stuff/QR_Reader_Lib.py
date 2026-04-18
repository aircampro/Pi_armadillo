"""
  Library containing the following QR Code readers 
  1. zxing-cpp (best one)
  2. OpenCV QRCodeDetector
  3. OpenCV WeChatQRCode (CNN-based, opencv-contrib)
  4. deqr (quirc / qrdec)
  ref :- https://github.com/Kazuhito00/python-qrcode-synthetic-benchmark/blob/main/03_benchmark.py

"""
import csv
import json
import os
import sys
import time
import cv2
import numpy as np
from tabulate import tabulate

def imread_safe(path):
    """safe imread"""
    buf = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)

# ============================================================
# Classes for various QR Readers
# ============================================================

class ReaderBase:
    name: str = "base"

    def setup(self):
        return True

    def decode(self, image_path: str) -> list[str]:
        raise NotImplementedError

class ZxingCppReader(ReaderBase):
    name = "zxing-cpp"

    def setup(self):
        try:
            import zxingcpp
            self._zxingcpp = zxingcpp
            return True
        except ImportError:
            return False

    def decode(self, image_path):
        img = imread_safe(image_path)
        if img is None:
            return []
        results = self._zxingcpp.read_barcodes(img)
        return [r.text for r in results if r.text]

class OpenCVQRReader(ReaderBase):
    name = "opencv-qr"

    def setup(self):
        self._detector = cv2.QRCodeDetector()
        return True

    def decode(self, image_path):
        img = imread_safe(image_path)
        if img is None:
            return []
        retval, decoded_info, points, straight_qrcode = self._detector.detectAndDecodeMulti(img)
        if not retval:
            return []
        return [d for d in decoded_info if d]

class WeChatQRReader(ReaderBase):
    name = "wechat-qr"

    def setup(self):
        try:
            model_dir = os.path.join(os.path.dirname(__file__), "models", "wechat_qrcode")
            detect_proto = os.path.join(model_dir, "detect.prototxt")
            detect_model = os.path.join(model_dir, "detect.caffemodel")
            sr_proto = os.path.join(model_dir, "sr.prototxt")
            sr_model = os.path.join(model_dir, "sr.caffemodel")

            if all(os.path.exists(f) for f in [detect_proto, detect_model, sr_proto, sr_model]):
                import shutil
                import tempfile
                tmp_model_dir = os.path.join(tempfile.gettempdir(), "wechat_qrcode_models")
                os.makedirs(tmp_model_dir, exist_ok=True)
                files_map = {}
                for src in [detect_proto, detect_model, sr_proto, sr_model]:
                    dst = os.path.join(tmp_model_dir, os.path.basename(src))
                    if not os.path.exists(dst):
                        shutil.copy2(src, dst)
                    files_map[os.path.basename(src)] = dst

                self._detector = cv2.wechat_qrcode_WeChatQRCode(
                    files_map["detect.prototxt"],
                    files_map["detect.caffemodel"],
                    files_map["sr.prototxt"],
                    files_map["sr.caffemodel"],
                )
            else:
                self._detector = cv2.wechat_qrcode_WeChatQRCode()
            return True
        except (AttributeError, Exception) as e:
            print(f"  [wechat-qr] exception: {e}")
            return False

    def decode(self, image_path):
        img = imread_safe(image_path)
        if img is None:
            return []
        results, points = self._detector.detectAndDecode(img)
        return [r for r in results if r]

class DeqrReader(ReaderBase):
    name = "deqr"

    def setup(self):
        try:
            import deqr
            self._deqr = deqr
            return True
        except ImportError:
            return False

    def decode(self, image_path):
        img = imread_safe(image_path)
        if img is None:
            return []
        try:
            decoder = self._deqr.QRdecDecoder()
            results = decoder.decode(img)
            texts = []
            for r in results:
                for entry in r.data_entries:
                    if entry.data:
                        texts.append(entry.data if isinstance(entry.data, str)
                                     else entry.data.decode("utf-8", errors="replace"))
            return texts
        except Exception:
            return []
