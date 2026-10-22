"""Abstract base class for object detectors (allows swapping YOLO versions)."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List
import numpy as np


@dataclass
class Detection:
    """Single detection result."""
    label: str
    confidence: float
    x_center: float   # pixels
    y_center: float   # pixels
    width: float      # pixels
    height: float     # pixels

    @property
    def x_left(self) -> float:
        return self.x_center - self.width / 2.0

    @property
    def x_right(self) -> float:
        return self.x_center + self.width / 2.0

    @property
    def y_top(self) -> float:
        return self.y_center - self.height / 2.0

    @property
    def y_bottom(self) -> float:
        return self.y_center + self.height / 2.0


class DetectorInterface(ABC):
    """Abstract detector — subclass for YOLOv8, YOLO-World, etc."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run inference on a BGR image, return a list of detections."""
        ...

    @abstractmethod
    def set_classes(self, classes: List[str]) -> None:
        """For open-vocabulary detectors: set the active class list."""
        ...
