#!/usr/bin/env python3
"""
object_classifier_node.py

Тот самый "отдельный ML-модуль", который команда упоминает в README как
недостающую часть: "для классификации типа объекта используется отдельный
ML-модуль". Геометрический pipeline (preprocessor -> gauge_filter ->
anomaly_detector) уже отделяет точки препятствия от фона тоннеля и
публикует их в /metro/obstacle_points — эта нода их забирает и определяет,
на что похоже само облако точек в кадре (класс сечения тоннеля /
platform / obstacle / pressureGate / switch, см. CLASSES).

Важно: сеть в текущем виде обучена на КАДРАХ ЦЕЛИКОМ (задача была
"классификация сегмента пути", т.к. в bag-файлах нет поточной разметки
отдельных препятствий, см. docs/experiments в нашей части — segment_labeling.py).
Она работает и как индикатор "на что похож текущий кадр", в т.ч. когда
anomaly_detector считает, что в кадре есть обструкция — тогда с высокой
вероятностью предсказанный класс будет "obstacle". Это честное текущее
ограничение, не попытка выдать больше, чем есть: датасет без per-object
разметки не позволяет обучить настоящий object-level классификатор без
изменения постановки. Подробности — в README PointNet-части.

Подписка:  /metro/obstacle_points  (sensor_msgs/PointCloud2, из anomaly_detector.cpp)
Публикация: /metro/object_class     (std_msgs/String, формат "id:name:confidence")

Веса модели: параметр model_path (по умолчанию ищет pointnet_lite_best.pt
рядом со скриптом). Если файла нет — нода стартует, логирует
предупреждение и просто ничего не публикует (остальной pipeline при этом
продолжает работать как ни в чём не бывало — эта нода строго опциональна).
"""

import os
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import torch
import torch.nn as nn
import torch.nn.functional as F

CLASSES = ["roundT", "squareT", "doubleT", "platform", "obstacle", "pressureGate", "switch"]
K_POINTS = 2048


class PointNetLite(nn.Module):
    """Должна побайтово совпадать с архитектурой из train.py — иначе
    state_dict не загрузится (или, хуже, загрузится молча неправильно)."""

    def __init__(self, n_classes: int = len(CLASSES)):
        super().__init__()
        self.mlp1 = nn.Linear(3, 64)
        self.mlp2 = nn.Linear(64, 128)
        self.mlp3 = nn.Linear(128, 256)
        self.head1 = nn.Linear(256, 128)
        self.head2 = nn.Linear(128, 64)
        self.head3 = nn.Linear(64, n_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = F.relu(self.mlp1(x))
        h = F.relu(self.mlp2(h))
        h = F.relu(self.mlp3(h))
        h = h.max(dim=1).values
        h = F.relu(self.head1(h))
        h = F.relu(self.head2(h))
        return self.head3(h)


def subsample(points_xyz: np.ndarray, k: int, rng: np.random.Generator) -> np.ndarray:
    n = len(points_xyz)
    if n == 0:
        return np.zeros((k, 3), dtype=np.float32)
    if n >= k:
        idx = rng.choice(n, size=k, replace=False)
    else:
        idx = rng.choice(n, size=k, replace=True)
    return points_xyz[idx]


class ObjectClassifierNode(Node):
    def __init__(self):
        super().__init__("object_classifier_node")

        default_model_path = str(Path(__file__).resolve().parent / "pointnet_lite_best.pt")
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("min_points", 10)
        self.declare_parameter("device", "cpu")  # "cuda" на стенде с RTX 4070 Ti, если нужно

        model_path = self.get_parameter("model_path").value
        self.min_points = int(self.get_parameter("min_points").value)
        device_name = self.get_parameter("device").value
        self.device = torch.device(device_name if torch.cuda.is_available() or device_name == "cpu" else "cpu")

        self.rng = np.random.default_rng(0)
        self.model = None

        if os.path.isfile(model_path):
            self.model = PointNetLite().to(self.device)
            state = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(state)
            self.model.eval()
            self.get_logger().info(f"object_classifier_node: веса загружены из {model_path} ({self.device})")
        else:
            self.get_logger().warning(
                f"object_classifier_node: файл весов не найден ({model_path}), "
                f"нода запущена, но классифицировать НЕЧЕМ — /metro/object_class публиковаться не будет. "
                f"Укажите параметр model_path на pointnet_lite_best.pt")

        self.pub = self.create_publisher(String, "/metro/object_class", 10)
        self.create_subscription(
            PointCloud2, "/metro/obstacle_points", self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, msg: PointCloud2):
        if self.model is None:
            return

        # В Humble read_points() возвращает структурированный массив numpy —
        # np.array(list(...), dtype=float32) на его записях (numpy.void) падает.
        # read_points_numpy() сразу даёт обычный массив (N, 3).
        pts = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True).astype(np.float32, copy=False)
        if pts.ndim != 2 or pts.shape[0] < self.min_points:
            return

        xyz = pts.copy()
        xyz[:, 1] -= np.median(xyz[:, 1])  # центрируем по Y, как при обучении (build_dataset.py)
        sub = subsample(xyz, K_POINTS, self.rng)

        with torch.no_grad():
            batch = torch.from_numpy(sub).unsqueeze(0).to(self.device)  # (1, K, 3)
            logits = self.model(batch)
            probs = F.softmax(logits, dim=-1).squeeze(0).cpu().numpy()

        class_id = int(np.argmax(probs))
        class_name = CLASSES[class_id]
        confidence = float(probs[class_id])

        out = String()
        out.data = f"{class_id}:{class_name}:{confidence:.4f}"
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObjectClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
