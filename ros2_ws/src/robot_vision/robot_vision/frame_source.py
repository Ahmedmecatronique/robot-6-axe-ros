"""Sources d'images directes (hors ROS) : webcam USB / GStreamer (OpenCV) et Pi Camera (Picamera2)."""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np


class FrameSource:
    """Interface commune : read() -> image BGR ou None ; close()."""

    def read(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class OpenCVSource(FrameSource):
    """Webcam USB (index « 0 ») ou pipeline GStreamer (chaîne contenant « ! »)."""

    def __init__(self, device: str = '0', width: int = 640, height: int = 480, fps: float = 30.0):
        if '!' in device:
            self.cap = cv2.VideoCapture(device, cv2.CAP_GSTREAMER)
        else:
            self.cap = cv2.VideoCapture(int(device) if device.isdigit() else device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f'Impossible d\'ouvrir la caméra « {device} »')

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


class PiCamera2Source(FrameSource):
    """Pi Camera v3 via Picamera2 (libcamera).

    La mise au point est FIXÉE (lens_position en dioptries = 1/distance en mètres ; 0 = infini) :
    l'autofocus modifie la focale effective et fausserait la calibration.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: float = 30.0, lens_position: float = 2.0):
        from picamera2 import Picamera2  # import tardif : module absent hors Raspberry Pi
        self.cam = Picamera2()
        config = self.cam.create_video_configuration(main={'size': (width, height), 'format': 'RGB888'})
        self.cam.configure(config)
        controls = {'FrameRate': float(fps)}
        try:  # Camera Module 3 : autofocus motorisé ; sur v2 ces contrôles n'existent pas
            controls.update({'AfMode': 0, 'LensPosition': float(lens_position)})
        except Exception:  # noqa: BLE001
            pass
        self.cam.set_controls(controls)
        self.cam.start()

    def read(self) -> Optional[np.ndarray]:
        # Le format "RGB888" de Picamera2 est en réalité ordonné B,G,R en mémoire : déjà du BGR pour OpenCV.
        return self.cam.capture_array()

    def close(self) -> None:
        self.cam.stop()
        self.cam.close()


def open_source(kind: str, device: str = '0', width: int = 640, height: int = 480, fps: float = 30.0,
                lens_position: float = 2.0) -> FrameSource:
    if kind == 'picamera2':
        return PiCamera2Source(width, height, fps, lens_position)
    if kind == 'opencv':
        return OpenCVSource(device, width, height, fps)
    raise ValueError(f'source inconnue : {kind}')
