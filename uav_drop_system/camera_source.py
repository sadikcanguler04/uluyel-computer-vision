"""
Picamera2 sarmalayıcısı.

Bu proje ağacında `picamera2` yalnızca bu dosyada import edilir. Diğer hiçbir
modül (vision/*, main.py dahil) doğrudan picamera2 import etmez; böylece
vision/mavlink/localization/drop/planning gibi saf mantık modülleri
Raspberry Pi olmayan bir geliştirme makinesinde (örn. Windows) picamera2
hiç kurulu olmasa bile import edilip test edilebilir.

old_docs/vision_servo_trigger.py'deki start_picamera2()/get_camera_frame()
davranışı, import zamanında değil `start()` çağrıldığında çalışacak şekilde
bir sınıfa taşınmıştır. Varsayılan format/config/timing eski kodla aynıdır.
"""

import time

import cv2


class CameraSource:
    """old_docs/vision_servo_trigger.py::start_picamera2 + get_camera_frame'in taşınmış hâli."""

    def __init__(self, width, height, fps, use_rgb_to_bgr=False, warmup_sec=2.0):
        self.width = width
        self.height = height
        self.fps = fps
        self.use_rgb_to_bgr = use_rgb_to_bgr
        self.warmup_sec = warmup_sec
        self._picam2 = None

    def start(self):
        from picamera2 import Picamera2

        picam2 = Picamera2()

        config = picam2.create_preview_configuration(
            main={
                "size": (self.width, self.height),
                "format": "RGB888"
            }
        )

        picam2.configure(config)
        picam2.start()
        time.sleep(self.warmup_sec)

        self._picam2 = picam2
        return self

    def get_frame(self):
        if self._picam2 is None:
            raise RuntimeError("CameraSource.start() çağrılmadan get_frame() kullanılamaz.")

        frame = self._picam2.capture_array()

        if self.use_rgb_to_bgr:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = frame.copy()

        return frame_bgr

    def stop(self):
        if self._picam2 is not None:
            self._picam2.stop()
            self._picam2 = None
