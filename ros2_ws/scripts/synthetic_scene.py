#!/usr/bin/env python3
"""
Генератор синтетических лидарных сцен для тестирования пайплайна.

Генерирует прямоугольный тоннель с опциональным препятствием на заданной
дистанции. Публикует в топик /lidar_points с частотой 10 Гц.

Использование:
    python3 synthetic_scene.py                          # пустой тоннель
    python3 synthetic_scene.py --distance 50            # препятствие на 50 м
    python3 synthetic_scene.py --distance 100 --size 0.5 --y 0.3
    python3 synthetic_scene.py --distance 20 --size 2.0 --duration 15
"""

import argparse
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def create_point_cloud2_msg(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Создаёт PointCloud2 из Nx3 или Nx4 массива (x, y, z[, intensity])."""
    if points.shape[1] == 3:
        intensity = np.full((points.shape[0], 1), 100.0, dtype=np.float32)
        points = np.hstack([points, intensity])

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


def generate_tunnel(
    length: float = 300.0,
    half_width: float = 2.5,
    height: float = 4.0,
    spacing_near: float = 0.15,
    spacing_far: float = 0.5,
    transition_dist: float = 30.0,
) -> np.ndarray:
    """Генерирует точки прямоугольного тоннеля (пол, стены, потолок)."""
    points = []

    # Адаптивный шаг: плотнее вблизи, реже вдали
    x_positions = []
    x = 0.5
    while x < length:
        x_positions.append(x)
        step = spacing_near if x < transition_dist else spacing_far
        x += step

    x_arr = np.array(x_positions)

    # Пол (Z = 0)
    for x in x_arr:
        spacing = spacing_near if x < transition_dist else spacing_far
        y_vals = np.arange(-half_width, half_width + spacing, spacing)
        for y in y_vals:
            points.append([x, y, 0.0])

    # Потолок (Z = height)
    for x in x_arr:
        spacing = spacing_near if x < transition_dist else spacing_far
        y_vals = np.arange(-half_width, half_width + spacing, spacing)
        for y in y_vals:
            points.append([x, y, height])

    # Стены (Y = ±half_width)
    for x in x_arr:
        spacing = spacing_near if x < transition_dist else spacing_far
        z_vals = np.arange(0, height + spacing, spacing)
        for z in z_vals:
            points.append([x, -half_width, z])
            points.append([x, half_width, z])

    return np.array(points, dtype=np.float32)


def generate_obstacle(
    distance: float,
    size: float = 1.0,
    y_offset: float = 0.0,
    height: float = 1.5,
    spacing: float = 0.08,
) -> np.ndarray:
    """Генерирует препятствие (прямоугольный блок) на заданной дистанции."""
    points = []
    half = size / 2.0

    # Передняя грань ( обращена к поезду)
    y_vals = np.arange(y_offset - half, y_offset + half + spacing, spacing)
    z_vals = np.arange(0.0, height + spacing, spacing)
    for y in y_vals:
        for z in z_vals:
            points.append([distance, y, z])

    # Верхняя грань
    x_vals = np.arange(distance, distance + size + spacing, spacing)
    for x in x_vals:
        for y in y_vals:
            points.append([x, y, height])

    # Боковые грани
    for x in x_vals:
        for z in z_vals:
            points.append([x, y_offset - half, z])
            points.append([x, y_offset + half, z])

    return np.array(points, dtype=np.float32)


class SyntheticScenePublisher(Node):
    def __init__(self, args):
        super().__init__("synthetic_scene_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher = self.create_publisher(PointCloud2, "/lidar_points", qos)

        self.frame_id = "lidar_livox"
        self.duration = args.duration
        self.rate_hz = 10.0
        self.frame_count = 0

        # Генерируем тоннель
        self.get_logger().info("Генерация тоннеля...")
        tunnel = generate_tunnel()
        self.get_logger().info(f"  Тоннель: {len(tunnel)} точек")

        # Опциональное препятствие
        self.obstacle = None
        if args.distance > 0:
            self.obstacle = generate_obstacle(
                distance=args.distance,
                size=args.size,
                y_offset=args.y,
            )
            self.get_logger().info(
                f"  Препятствие: {len(self.obstacle)} точек, "
                f"дистанция={args.distance} м, размер={args.size} м, y={args.y} м"
            )
            self.cloud = np.vstack([tunnel, self.obstacle])
        else:
            self.get_logger().info("  Препятствие: отсутствует (пустой тоннель)")
            self.cloud = tunnel

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_frame)
        self.start_time = time.time()
        self.get_logger().info(
            f"Публикация: {len(self.cloud)} точек/кадр, {self.rate_hz} Гц, "
            f"длительность {self.duration} с"
        )

    def publish_frame(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.get_logger().info("Синтетическая сцена завершена.")
            raise SystemExit()

        stamp = self.get_clock().now().to_msg()
        msg = create_point_cloud2_msg(self.cloud, self.frame_id, stamp)
        self.publisher.publish(msg)
        self.frame_count += 1


def main():
    parser = argparse.ArgumentParser(description="Генератор синтетических лидарных сцен")
    parser.add_argument("--distance", type=float, default=0.0,
                        help="Дистанция до препятствия в метрах (0 = пустой тоннель)")
    parser.add_argument("--size", type=float, default=1.0,
                        help="Размер препятствия в метрах (ширина и глубина)")
    parser.add_argument("--y", type=float, default=0.0,
                        help="Смещение препятствия по Y (0 = центр)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Длительность публикации в секундах")
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticScenePublisher(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
