"""
Генератор синтетических лидарных сцен с несколькими целями разной формы.

Формы объектов:
  cube       - прямоугольный блок
  cylinder   - вертикальный цилиндр (столб, бревно)
  sphere     - сфера (мяч, камень)
  cone       - конус (дорожный конус)
  person     - человек (цилиндр + сфера)
  platform   - низкая широкая платформа (ящик, поддон)

Сценарии:
  single_cube  - один куб (обратная совместимость)
  multi_static - несколько статичных объектов разной формы
  moving       - несколько движущихся объектов
  mixed        - смесь статичных и движущихся

Использование:
  python3 synthetic_scene.py --save-ply scene.ply --scenario multi_static
  python3 synthetic_scene.py --scenario moving --duration 15
  python3 synthetic_scene.py --distance 50 --size 1.0   # как раньше
  python3 synthetic_scene.py --scenario mixed --speed 8 --duration 12
"""
import argparse
import time
import numpy as np

FRAME_ID = "lidar_livox"
TUNNEL_LENGTH = 300.0
HALF_WIDTH = 2.5
CEILING_HEIGHT = 4.0
TRANSITION_DIST = 30.0
SPACING_NEAR = 0.15
SPACING_FAR = 0.5

# ============================================================
# Генерация тоннеля (без изменений)
# ============================================================

def x_positions(length=TUNNEL_LENGTH):
    xs = []
    x = 0.5
    while x < length:
        xs.append(x)
        x += SPACING_NEAR if x < TRANSITION_DIST else SPACING_FAR
    return np.asarray(xs, dtype=np.float32)

def _with_noise(arr, noise, rng):
    if noise > 0 and arr.size:
        arr = arr + rng.uniform(-noise, noise, arr.shape).astype(np.float32)
    return arr

def make_floor(xs, noise, rng):
    pts = []
    for chunk, sp in ((xs[xs < TRANSITION_DIST], SPACING_NEAR),
                      (xs[xs >= TRANSITION_DIST], SPACING_FAR)):
        if chunk.size == 0: continue
        ys = np.arange(-HALF_WIDTH, HALF_WIDTH + sp, sp, dtype=np.float32)
        X, Y = np.meshgrid(chunk, ys, indexing="ij")
        pts.append(_with_noise(
            np.stack([X.ravel(), Y.ravel(), np.zeros(X.size, dtype=np.float32)], axis=1),
            noise, rng))
    return np.vstack(pts) if pts else np.zeros((0, 3), dtype=np.float32)

def make_ceiling(xs, noise, rng):
    pts = []
    for chunk, sp in ((xs[xs < TRANSITION_DIST], SPACING_NEAR),
                      (xs[xs >= TRANSITION_DIST], SPACING_FAR)):
        if chunk.size == 0: continue
        ys = np.arange(-HALF_WIDTH, HALF_WIDTH + sp, sp, dtype=np.float32)
        X, Y = np.meshgrid(chunk, ys, indexing="ij")
        Z = np.full(X.size, CEILING_HEIGHT, dtype=np.float32)
        pts.append(_with_noise(
            np.stack([X.ravel(), Y.ravel(), Z], axis=1), noise, rng))
    return np.vstack(pts) if pts else np.zeros((0, 3), dtype=np.float32)

def make_walls(xs, noise, rng):
    pts = []
    for chunk, sp in ((xs[xs < TRANSITION_DIST], SPACING_NEAR),
                      (xs[xs >= TRANSITION_DIST], SPACING_FAR)):
        if chunk.size == 0: continue
        zs = np.arange(0.0, CEILING_HEIGHT + sp, sp, dtype=np.float32)
        X, Z = np.meshgrid(chunk, zs, indexing="ij")
        for y in (-HALF_WIDTH, HALF_WIDTH):
            Y = np.full(X.size, y, dtype=np.float32)
            pts.append(_with_noise(
                np.stack([X.ravel(), Y, Z.ravel()], axis=1), noise, rng))
    return np.vstack(pts) if pts else np.zeros((0, 3), dtype=np.float32)

def make_rails(xs):
    pts = []
    for y_rail in (-0.76, 0.76):
        for dy, z in ((0.0, 0.16), (-0.04, 0.05), (0.04, 0.05)):
            Y = np.full(xs.shape, y_rail + dy, dtype=np.float32)
            Z = np.full(xs.shape, z, dtype=np.float32)
            pts.append(np.stack([xs, Y, Z], axis=1))
    return np.vstack(pts).astype(np.float32) if pts else np.zeros((0, 3), dtype=np.float32)

def make_sleepers(length=TUNNEL_LENGTH, step=0.6):
    pts = []
    ys = np.arange(-1.0, 1.01, 0.25, dtype=np.float32)
    for x in np.arange(1.0, length, step, dtype=np.float32):
        for y in ys:
            pts.append((x, y, 0.08))
    return np.asarray(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)

def make_cables(xs):
    pts = []
    for y in (-HALF_WIDTH + 0.15, HALF_WIDTH - 0.15):
        for z in (2.0, 2.8):
            Y = np.full(xs.shape, y, dtype=np.float32)
            Z = np.full(xs.shape, z, dtype=np.float32)
            pts.append(np.stack([xs, Y, Z], axis=1))
    return np.vstack(pts).astype(np.float32) if pts else np.zeros((0, 3), dtype=np.float32)

def make_rings(length=TUNNEL_LENGTH, step=6.0, inset=0.06):
    pts = []
    zs = np.arange(0.0, CEILING_HEIGHT + 0.3, 0.3, dtype=np.float32)
    ys = np.arange(-HALF_WIDTH, HALF_WIDTH + 0.3, 0.3, dtype=np.float32)
    for x in np.arange(step, length, step, dtype=np.float32):
        for z in zs:
            pts.append((x, -HALF_WIDTH + inset, z))
            pts.append((x, HALF_WIDTH - inset, z))
        for y in ys:
            pts.append((x, y, CEILING_HEIGHT - inset))
    return np.asarray(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)

# ============================================================
# Формы объектов (генерируются в локальных координатах, центр в 0,0,0)
# ============================================================

def shape_cube(size_x=1.0, size_y=1.0, size_z=1.5, spacing=0.08):
    """Прямоугольный блок. Центр основания в (0,0,0)."""
    pts = []
    hx, hy = size_x / 2, size_y / 2
    xs = np.arange(-hx, hx + spacing, spacing)
    ys = np.arange(-hy, hy + spacing, spacing)
    zs = np.arange(0, size_z + spacing, spacing)
    # Передняя и задняя грани
    for x in (-hx, hx):
        for y in ys:
            for z in zs:
                pts.append((x, y, z))
    # Боковые грани
    for y in (-hy, hy):
        for x in xs:
            for z in zs:
                pts.append((x, y, z))
    # Верхняя грань
    for x in xs:
        for y in ys:
            pts.append((x, y, size_z))
    return np.asarray(pts, dtype=np.float32)

def shape_cylinder(radius=0.3, height=1.5, spacing=0.08):
    """Вертикальный цилиндр. Центр основания в (0,0,0)."""
    pts = []
    # Боковая поверхность
    n_angle = max(8, int(2 * np.pi * radius / spacing))
    n_height = max(4, int(height / spacing))
    for i in range(n_height + 1):
        z = height * i / n_height
        for j in range(n_angle):
            angle = 2 * np.pi * j / n_angle
            pts.append((radius * np.cos(angle), radius * np.sin(angle), z))
    # Крышки
    n_r = max(2, int(radius / spacing))
    for i in range(n_r + 1):
        r = radius * i / n_r
        for j in range(n_angle):
            angle = 2 * np.pi * j / n_angle
            pts.append((r * np.cos(angle), r * np.sin(angle), 0.0))
            pts.append((r * np.cos(angle), r * np.sin(angle), height))
    return np.asarray(pts, dtype=np.float32)

def shape_sphere(radius=0.4, spacing=0.08):
    """Сфера. Центр в (0,0,radius) - стоит на земле."""
    pts = []
    n_phi = max(6, int(np.pi * radius / spacing))
    n_theta = max(8, int(2 * np.pi * radius / spacing))
    for i in range(n_phi + 1):
        phi = np.pi * i / n_phi
        for j in range(n_theta):
            theta = 2 * np.pi * j / n_theta
            pts.append((radius * np.sin(phi) * np.cos(theta),
                        radius * np.sin(phi) * np.sin(theta),
                        radius + radius * np.cos(phi)))
    return np.asarray(pts, dtype=np.float32)

def shape_cone(radius=0.3, height=0.7, spacing=0.08):
    """Конус (дорожный конус). Центр основания в (0,0,0)."""
    pts = []
    n_angle = max(8, int(2 * np.pi * radius / spacing))
    n_height = max(4, int(height / spacing))
    # Боковая поверхность
    for i in range(n_height + 1):
        z = height * i / n_height
        r = radius * (1 - z / height)
        for j in range(n_angle):
            angle = 2 * np.pi * j / n_angle
            pts.append((r * np.cos(angle), r * np.sin(angle), z))
    # Основание
    n_r = max(2, int(radius / spacing))
    for i in range(n_r + 1):
        r = radius * i / n_r
        for j in range(n_angle):
            angle = 2 * np.pi * j / n_angle
            pts.append((r * np.cos(angle), r * np.sin(angle), 0.0))
    return np.asarray(pts, dtype=np.float32)

def shape_person(spacing=0.08):
    """Человек: цилиндр (тело) + сфера (голова). Ноги на земле."""
    body = shape_cylinder(radius=0.22, height=1.25, spacing=spacing)
    head = shape_sphere(radius=0.14, spacing=spacing)
    head[:, 2] += 1.25  # голова на высоте 1.25 м
    return np.vstack([body, head]).astype(np.float32)

def shape_platform(length=1.5, width=1.0, height=0.4, spacing=0.08):
    """Низкая широкая платформа (ящик, поддон). Центр основания в (0,0,0)."""
    pts = []
    hx, hy = length / 2, width / 2
    xs = np.arange(-hx, hx + spacing, spacing)
    ys = np.arange(-hy, hy + spacing, spacing)
    zs = np.arange(0, height + spacing, spacing)
    # 4 боковые грани
    for x in (-hx, hx):
        for y in ys:
            for z in zs:
                pts.append((x, y, z))
    for y in (-hy, hy):
        for x in xs:
            for z in zs:
                pts.append((x, y, z))
    # Верхняя грань
    for x in xs:
        for y in ys:
            pts.append((x, y, height))
    return np.asarray(pts, dtype=np.float32)

SHAPE_FUNCS = {
    "cube": shape_cube,
    "cylinder": shape_cylinder,
    "sphere": shape_sphere,
    "cone": shape_cone,
    "person": shape_person,
    "platform": shape_platform,
}

# ============================================================
# Объект сцены (статичный или движущийся)
# ============================================================

class SceneObject:
    """Объект в сцене. Может быть статичным или двигаться с постоянной скоростью."""
    def __init__(self, shape, x, y, z=0.0, vx=0.0, vy=0.0, vz=0.0, intensity=150.0, **params):
        self.shape = shape
        self.x0, self.y0, self.z0 = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz
        self.intensity = intensity
        self.name = params.get("name", shape)
        # Генерируем локальные точки один раз
        func = SHAPE_FUNCS[shape]
        clean_params = {k: v for k, v in params.items() if k != "name"}
        self.local_points = func(**clean_params)
        self.is_moving = (vx != 0 or vy != 0 or vz != 0)

    def get_points(self, t=0.0):
        """Возвращает точки объекта в момент времени t (мировые координаты)."""
        pts = self.local_points.copy()
        pts[:, 0] += self.x0 + self.vx * t
        pts[:, 1] += self.y0 + self.vy * t
        pts[:, 2] += self.z0 + self.vz * t
        return pts

    def get_points_with_intensity(self, t=0.0):
        """Точки + колонка intensity."""
        pts = self.get_points(t)
        col = np.full((pts.shape[0], 1), self.intensity, dtype=np.float32)
        return np.hstack([pts, col])

    def describe(self):
        motion = ""
        if self.is_moving:
            motion = f", v=({self.vx:+.1f},{self.vy:+.1f},{self.vz:+.1f}) м/с"
        return (f"{self.name}[{self.shape}] pos=({self.x0:.0f},{self.y0:.1f},{self.z0:.1f})"
                f"{motion}")

# ============================================================
# Предопределённые сценарии
# ============================================================

def scenario_single_cube(distance, size, y):
    """Один куб (обратная совместимость)."""
    return [SceneObject("cube", distance, y, 0.0,
                        size_x=size, size_y=size, size_z=1.5,
                        name="obstacle")]

def scenario_multi_static():
    """Несколько статичных объектов разной формы на разных дистанциях."""
    return [
        SceneObject("cube", 25, 0.0, size_x=1.0, size_y=1.0, size_z=1.5,
                    intensity=150.0, name="куб"),
        SceneObject("cylinder", 45, -0.6, radius=0.3, height=1.6,
                    intensity=160.0, name="цилиндр"),
        SceneObject("sphere", 65, 0.5, radius=0.45,
                    intensity=140.0, name="сфера"),
        SceneObject("person", 85, 0.0,
                    intensity=180.0, name="человек"),
        SceneObject("cone", 105, -0.8, radius=0.3, height=0.7,
                    intensity=170.0, name="конус"),
        SceneObject("platform", 125, 0.3, length=1.5, width=1.0, height=0.4,
                    intensity=130.0, name="платформа"),
    ]

def scenario_moving():
    """Несколько движущихся объектов (пара статичных для контраста)."""
    return [
        # Движущиеся
        SceneObject("cube", 40, 0.0, vx=-3.0,
                    size_x=1.0, size_y=1.0, size_z=1.5,
                    intensity=150.0, name="куб_движ"),
        SceneObject("person", 70, 0.8, vx=-2.0, vy=-0.3,
                    intensity=180.0, name="человек_движ"),
        SceneObject("cylinder", 90, -1.2, vy=0.8,
                    radius=0.3, height=1.5,
                    intensity=160.0, name="цилиндр_движ"),
        SceneObject("sphere", 55, 0.0, z=0.5, vx=-4.0, vy=0.5,
                    radius=0.35,
                    intensity=140.0, name="сфера_движ"),
        # Статичные для контраста
        SceneObject("cone", 110, 0.5, radius=0.3, height=0.7,
                    intensity=170.0, name="конус_стат"),
    ]

def scenario_mixed():
    """Смесь: 3 статичных + 3 движущихся объекта разной формы."""
    return [
        # Статичные
        SceneObject("cube", 30, -0.5, size_x=1.2, size_y=0.8, size_z=1.5,
                    intensity=150.0, name="куб_стат"),
        SceneObject("platform", 60, 0.6, length=1.8, width=1.2, height=0.5,
                    intensity=130.0, name="платформа_стат"),
        SceneObject("cone", 90, -0.9, radius=0.35, height=0.8,
                    intensity=170.0, name="конус_стат"),
        # Движущиеся
        SceneObject("person", 50, 0.0, vx=-2.5,
                    intensity=180.0, name="человек_движ"),
        SceneObject("cylinder", 80, 1.0, vx=-1.5, vy=-0.4,
                    radius=0.25, height=1.4,
                    intensity=160.0, name="цилиндр_движ"),
        SceneObject("sphere", 110, -0.3, z=0.4, vx=-3.5,
                    radius=0.4,
                    intensity=140.0, name="сфера_движ"),
    ]

SCENARIOS = {
    "single_cube": None,  # обрабатывается отдельно
    "multi_static": scenario_multi_static,
    "moving": scenario_moving,
    "mixed": scenario_mixed,
}

# ============================================================
# Вспомогательные функции
# ============================================================

def build_tunnel(args):
    """Строит статичный тоннель (пол, потолок, стены, рельсы и т.д.)."""
    rng = np.random.default_rng(42)
    xs = x_positions()
    parts = [
        (make_floor(xs, args.noise, rng), 80.0),
        (make_ceiling(xs, args.noise, rng), 90.0),
        (make_walls(xs, args.noise, rng), 100.0),
        (make_rails(xs), 200.0),
        (make_sleepers(), 70.0),
        (make_cables(xs), 60.0),
        (make_rings(), 110.0),
    ]
    out = []
    for arr, intensity in parts:
        if arr.size == 0: continue
        col = np.full((arr.shape[0], 1), intensity, dtype=np.float32)
        out.append(np.hstack([arr[:, :3], col]))
    return np.vstack(out).astype(np.float32) if out else np.zeros((0, 4), dtype=np.float32)

def build_objects(args):
    """Создаёт список объектов по сценарию."""
    if args.scenario == "single_cube":
        if args.distance <= 0:
            return []
        return scenario_single_cube(args.distance, args.size, args.y)
    factory = SCENARIOS.get(args.scenario)
    if factory is None:
        raise ValueError(f"Неизвестный сценарий: {args.scenario}")
    return factory()

def save_ply(filename, points_nx4):
    n = points_nx4.shape[0]
    with open(filename, "wb") as f:
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {n}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property float intensity\n"
            "end_header\n"
        )
        f.write(header.encode("ascii"))
        f.write(points_nx4.astype(np.float32).tobytes())
    print(f"Сохранено: {filename} ({n} точек, {(n*16)/(1024*1024):.2f} MiB)")

# ============================================================
# ROS2 публикация (ленивые импорты)
# ============================================================

def run_publisher(args, tunnel_cloud, objects):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, PointField

    def to_pointcloud2(points_nx4, stamp):
        msg = PointCloud2()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = points_nx4.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * msg.width
        msg.is_dense = True
        msg.data = points_nx4.tobytes()
        return msg

    class Publisher(Node):
        def __init__(self):
            super().__init__("synthetic_scene_publisher")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
            self.publisher = self.create_publisher(PointCloud2, "/lidar_points", qos)
            self.train_speed = args.speed
            self.rate_hz = 10.0
            self.duration = args.duration
            self.frame = 0
            self.start_time = time.time()
            self.tunnel = tunnel_cloud
            self.objects = objects
            n_moving = sum(1 for o in objects if o.is_moving)
            self.get_logger().info(
                f"Публикация: тоннель={tunnel_cloud.shape[0]} точек, "
                f"объектов={len(objects)} (движущихся={n_moving}), "
                f"{self.rate_hz} Гц, {self.duration} с, скорость поезда {self.train_speed} м/с")
            for obj in objects:
                self.get_logger().info(f"  {obj.describe()}")
            self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_frame)

        def publish_frame(self):
            elapsed = time.time() - self.start_time
            if elapsed > self.duration:
                self.get_logger().info("Синтетическая сцена завершена.")
                raise SystemExit()
            t = self.frame / self.rate_hz
            # Движение поезда: сдвигаем весь мир назад
            train_offset = self.train_speed * t
            parts = [self.tunnel.copy()]
            parts[0][:, 0] -= train_offset
            # Объекты: пересчитываем позиции на момент t
            for obj in self.objects:
                pts = obj.get_points_with_intensity(t)
                pts[:, 0] -= train_offset
                parts.append(pts)
            cloud = np.vstack(parts)
            # Удаляем точки позади лидара
            cloud = cloud[cloud[:, 0] > 0.3]
            msg = to_pointcloud2(cloud, self.get_clock().now().to_msg())
            self.publisher.publish(msg)
            self.frame += 1

    rclpy.init()
    node = Publisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

# ============================================================
# main
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Синтетические лидарные сцены с несколькими целями")
    parser.add_argument("--scenario", type=str, default="single_cube",
                        choices=list(SCENARIOS.keys()),
                        help="Сценарий: single_cube, multi_static, moving, mixed")
    parser.add_argument("--distance", type=float, default=0.0,
                        help="Дистанция до куба (только для single_cube)")
    parser.add_argument("--size", type=float, default=1.0, help="Размер куба (single_cube)")
    parser.add_argument("--y", type=float, default=0.0, help="Смещение куба по Y (single_cube)")
    parser.add_argument("--duration", type=float, default=10.0, help="Длительность, с")
    parser.add_argument("--speed", type=float, default=0.0, help="Скорость поезда, м/с")
    parser.add_argument("--noise", type=float, default=0.02, help="Шум поверхности, м")
    parser.add_argument("--save-ply", type=str, default=None,
                        help="Сохранить сцену в PLY (t=0) и выйти без ROS2")
    args = parser.parse_args()

    print("Генерация тоннеля...")
    tunnel = build_tunnel(args)
    objects = build_objects(args)

    print(f"Объектов в сцене: {len(objects)}")
    for obj in objects:
        print(f"  {obj.describe()}")

    if args.save_ply:
        parts = [tunnel]
        for obj in objects:
            parts.append(obj.get_points_with_intensity(0.0))
        cloud = np.vstack(parts) if parts else np.zeros((0, 4), dtype=np.float32)
        save_ply(args.save_ply, cloud)
        return

    run_publisher(args, tunnel, objects)

if __name__ == "__main__":
    main()
