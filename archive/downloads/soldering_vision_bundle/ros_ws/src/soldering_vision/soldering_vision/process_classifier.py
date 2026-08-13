"""ConvNeXt process-state classifier loaded from a local checkpoint."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


DEFAULT_PROCESS_CLASSES = (
    "ready",
    "contact_good",
    "solder_good",
    "insufficient",
    "excess",
    "misaligned",
    "occluded",
    "unsafe",
)


@dataclass(frozen=True)
class Classification:
    label: str = "unknown"
    confidence: float = 0.0
    probabilities: tuple[float, ...] = ()


class DisabledClassifier:
    enabled = False

    def classify(self, _crop_bgr: np.ndarray) -> Classification:
        return Classification()


class ConvNextProcessClassifier:
    enabled = True

    def __init__(
        self,
        checkpoint_path: str,
        *,
        device: str = "cuda",
        classes: tuple[str, ...] = DEFAULT_PROCESS_CLASSES,
    ) -> None:
        if not checkpoint_path or not Path(checkpoint_path).exists():
            raise FileNotFoundError(
                f"ConvNeXt checkpoint not found: {checkpoint_path}"
            )
        import torch
        from torchvision.models import convnext_tiny

        self.torch = torch
        selected_device = device
        if device.startswith("cuda") and not torch.cuda.is_available():
            selected_device = "cpu"
        self.device = torch.device(selected_device)
        checkpoint = torch.load(
            checkpoint_path,
            map_location="cpu",
            weights_only=True,
        )
        if isinstance(checkpoint, dict) and "classes" in checkpoint:
            classes = tuple(str(item) for item in checkpoint["classes"])
        state_dict = (
            checkpoint["state_dict"]
            if isinstance(checkpoint, dict) and "state_dict" in checkpoint
            else checkpoint
        )
        self.classes = classes
        self.model = convnext_tiny(weights=None, num_classes=len(classes))
        self.model.load_state_dict(state_dict)
        self.model.eval().to(self.device)
        self.mean = torch.tensor(
            [0.485, 0.456, 0.406], device=self.device
        ).view(1, 3, 1, 1)
        self.std = torch.tensor(
            [0.229, 0.224, 0.225], device=self.device
        ).view(1, 3, 1, 1)

    def classify(self, crop_bgr: np.ndarray) -> Classification:
        if crop_bgr.size == 0:
            return Classification()
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_AREA)
        tensor = self.torch.from_numpy(resized).to(self.device)
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).float() / 255.0
        tensor = (tensor - self.mean) / self.std
        with self.torch.inference_mode():
            probabilities = self.torch.softmax(self.model(tensor), dim=1)[0]
        confidence, index = probabilities.max(dim=0)
        return Classification(
            label=self.classes[int(index)],
            confidence=float(confidence),
            probabilities=tuple(float(value) for value in probabilities),
        )
