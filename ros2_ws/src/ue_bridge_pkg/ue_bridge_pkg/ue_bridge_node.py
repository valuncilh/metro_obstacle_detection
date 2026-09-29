#!/usr/bin/env python3
"""
ue_bridge_node.py

Мост между UE5-симулятором (Windows) и реальным ROS2-пайплайном обнаружения
препятствий (metro_obstacle_detection, ROS2 Humble).

Роль ноды:
  1. Принимает по TCP от UE5 кадры облака точек (сгенерированные процедурно
     или взятые из bag-плеера внутри UE) и публикует их как самый обычный
     sensor_msgs/msg/PointCloud2 на топик /lidar_points — ровно то, что
     слушает preprocessor.cpp команды. Никакой логики обнаружения здесь нет
     и не должно быть: вся "суть определения" остаётся в их пайплайне.
  2. Подписывается на результат их пайплайна (/metro/safety_status,
     /metro/obstacle_distance) и, если запущена наша classifier-нода —
     на /metro/object_class, и пересылает всё это обратно в UE по тому же
     TCP-соединению, чтобы панель ALGORITHM могла это отрисовать.

Эта нода НЕ входит в их репозиторий и не меняет ни одного их файла —
запускается рядом, отдельным процессом/пакетом. Для реальной сдачи
хакатона она не нужна вообще (жюри гоняет их пайплайн на настоящих bag-
файлах через `ros2 bag play`), она нужна только вам для связки UE <-> ROS2
на этапе разработки/тестирования/визуализации.

Протокол TCP (простой, бинарный, little-endian; не ROS2-wire, свой):

  UE5 -> bridge (кадр лидара):
    header: <4sII>  magic=b'ULDR', msg_type=1, payload_len
    payload: <dI>    timestamp_sec (float64), num_points (uint32)
             затем num_points раз: <ffff> x, y, z, intensity (float32)
             (система ROS: метры, X вперёд, Y влево, Z вверх)

  bridge -> UE5 (результат алгоритма):
    header: <4sII>  magic=b'ULDR', msg_type=2, payload_len (=42)
    payload: <BfB32sf>
             status      uint8   0=SAFE, 1=DANGER, 255=нет данных ещё
             distance    float32 (-1.0, если нет объекта)
             class_id    uint8   255 = класс не определён / classifier-нода не запущена
             class_name  32 bytes, ASCII, дополнено нулями
             confidence  float32 (0..1)

Запуск (после colcon build в ros2_ws, где лежит этот пакет):
  ros2 run ue_bridge_pkg ue_bridge_node --ros-args -p tcp_port:=9999

Либо напрямую, без colcon (ROS2 env уже source'нут):
  python3 ue_bridge_node.py

Что увидите в консоли:
  [INFO] слушаю UE5 на 0.0.0.0:9999 ...          — нода запущена
  [INFO] ===== UE5 ПОДКЛЮЧИЛСЯ: 192.168.x.x ===== — UE подключился
  [INFO] первый кадр от UE5: N точек -> /lidar_points
  [INFO] UE5 -> ROS2: 10.0 кадр/с, ~50000 точек/кадр, ... (каждые stats_period_sec)
  [WARN] UE5 отключился: ...                       — соединение разорвано
"""

import array
import socket
import struct
import threading
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Float32, Header
from sensor_msgs.msg import PointCloud2, PointField

MAGIC = b"ULDR"
MSG_TYPE_LIDAR_FRAME = 1
MSG_TYPE_ALGORITHM_RESULT = 2

HEADER_FMT = "<4sII"
HEADER_LEN = struct.calcsize(HEADER_FMT)

FRAME_PREFIX_FMT = "<dI"
FRAME_PREFIX_LEN = struct.calcsize(FRAME_PREFIX_FMT)
POINT_STEP = 16  # <ffff> x, y, z, intensity

RESULT_FMT = "<BfB32sf"
RESULT_LEN = struct.calcsize(RESULT_FMT)

# Кадр больше этого — почти наверняка битый поток, а не настоящее облако.
MAX_PAYLOAD_BYTES = 256 * 1024 * 1024

STATUS_SAFE = 0
STATUS_DANGER = 1
STATUS_UNKNOWN = 255

# Раскладка точки в сообщении PointCloud2 совпадает с раскладкой в TCP-кадре
# (4 × float32 подряд) — поэтому байты точек идут в msg.data как есть, без
# разбора по одной точке в Python (50+ тыс. точек × 10 Гц иначе не успевают).
POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    """Читает ровно n байт из TCP-сокета (recv может вернуть меньше)."""
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        read = sock.recv_into(view[got:], n - got)
        if read == 0:
            raise ConnectionError("соединение закрыто со стороны UE5")
        got += read
    return bytes(buf)


class UeBridgeNode(Node):
    def __init__(self):
        super().__init__("ue_bridge_node")

        self.declare_parameter("tcp_host", "0.0.0.0")
        self.declare_parameter("tcp_port", 9999)
        self.declare_parameter("lidar_topic", "/lidar_points")
        self.declare_parameter("frame_id", "hesai_lidar")
        # "ros" — штамп времени ROS в момент публикации (по умолчанию: время
        # симуляции UE начинается с нуля, и узлы, сверяющие время с TF/часами,
        # могли бы отбрасывать такие кадры как "очень старые").
        # "ue" — время кадра из UE (секунды с начала уровня).
        self.declare_parameter("stamp_source", "ros")
        self.declare_parameter("stats_period_sec", 5.0)

        self.tcp_host = self.get_parameter("tcp_host").value
        self.tcp_port = int(self.get_parameter("tcp_port").value)
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.stamp_source = str(self.get_parameter("stamp_source").value).lower()
        self.stats_period = float(self.get_parameter("stats_period_sec").value)

        self.lidar_pub = self.create_publisher(
            PointCloud2, self.lidar_topic, qos_profile_sensor_data)

        self.create_subscription(
            String, "/metro/safety_status", self._on_safety_status, 10)
        self.create_subscription(
            Float32, "/metro/obstacle_distance", self._on_obstacle_distance, 10)
        # object_classifier_node публикует сюда "class_id:class_name:confidence";
        # если нода не запущена — просто не придёт, и в UE уйдёт class_id=255.
        self.create_subscription(
            String, "/metro/object_class", self._on_object_class, 10)

        self._status = STATUS_UNKNOWN
        self._distance = -1.0
        self._class_id = 255
        self._class_name = ""
        self._confidence = 0.0
        self._result_lock = threading.Lock()

        self._client_sock = None
        self._client_addr = None
        self._client_lock = threading.Lock()
        self._server_sock = None

        self._frame_queue: "queue.Queue[tuple]" = queue.Queue(maxsize=4)

        # Статистика (обновляется из разных потоков — под замком).
        self._stats_lock = threading.Lock()
        self._stats = self._empty_stats()
        self._first_frame_logged = False
        self._first_result_logged = False

        # Отправка результата в UE — из отдельного потока, НЕ из колбэка rclpy.
        # Блокирующий sendall внутри executor'а при заполнении TCP-окна вешает
        # поток спиннера, rmw-очередь портится и на следующем take_message
        # вылетает "Unable to convert call argument to Python object".
        # Колбэки кладут только последнее состояние (drop-older): панели UE5
        # не нужен backlog, нужно свежее.
        self._out_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=1)
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()

        # Публикация — из таймера executor'а, а не из сокет-потока: весь
        # ROS2-код (publish) остаётся в потоке rclpy.
        self.create_timer(1.0 / 60.0, self._publish_pending_frames)
        if self.stats_period > 0:
            self.create_timer(self.stats_period, self._log_stats)

        self._server_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f"слушаю UE5 на {self.tcp_host}:{self.tcp_port}, публикую облако в "
            f"{self.lidar_topic} (frame_id={self.frame_id}, штамп времени: {self.stamp_source}). "
            f"Жду подключения...")

    @staticmethod
    def _empty_stats():
        return {"received": 0, "published": 0, "dropped": 0, "points": 0,
                "bytes": 0, "results_sent": 0, "t0": time.monotonic()}

    # ------------------------------------------------------------------ #
    # TCP сервер: ОДНО подключение от UE5 за раз (переподключение
    # допустимо — старое просто разорвётся, и примем новое).
    # ------------------------------------------------------------------ #
    def _accept_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.tcp_host, self.tcp_port))
        except OSError as e:
            # Без этого поток молча умирал бы, а нода выглядела бы "запущенной".
            self.get_logger().error(
                f"не могу открыть порт {self.tcp_host}:{self.tcp_port}: {e}. "
                f"Порт занят (ещё одна копия ноды?) — остановите её или укажите -p tcp_port:=...")
            return
        srv.listen(1)
        self._server_sock = srv

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info(f"===== UE5 ПОДКЛЮЧИЛСЯ: {addr[0]}:{addr[1]} =====")
            with self._client_lock:
                self._client_sock = conn
                self._client_addr = addr
            self._first_frame_logged = False
            with self._stats_lock:
                self._stats = self._empty_stats()
            # Сразу отправляем текущее состояние, чтобы панель в UE ожила,
            # даже если пайплайн ещё ничего не опубликовал.
            self._send_result_to_ue()
            try:
                self._client_loop(conn)
            except (ConnectionError, OSError) as e:
                self.get_logger().warning(f"UE5 отключился: {e}. Жду нового подключения...")
            finally:
                with self._client_lock:
                    if self._client_sock is conn:
                        self._client_sock = None
                        self._client_addr = None
                try:
                    conn.close()
                except OSError:
                    pass

    def _client_loop(self, conn: socket.socket):
        while rclpy.ok():
            header = _recv_exact(conn, HEADER_LEN)
            magic, msg_type, payload_len = struct.unpack(HEADER_FMT, header)
            if magic != MAGIC:
                raise ConnectionError(f"неверный magic в заголовке: {magic!r} (это не UE5-мост?)")
            if payload_len > MAX_PAYLOAD_BYTES:
                raise ConnectionError(f"слишком большой пакет: {payload_len} байт — поток повреждён")
            payload = _recv_exact(conn, payload_len)

            if msg_type == MSG_TYPE_LIDAR_FRAME:
                self._handle_lidar_frame(payload)
            else:
                self.get_logger().warning(f"неизвестный msg_type={msg_type}, игнорирую")

    def _handle_lidar_frame(self, payload: bytes):
        if len(payload) < FRAME_PREFIX_LEN:
            self.get_logger().warning("короткий кадр без заголовка — пропускаю")
            return
        timestamp_sec, num_points = struct.unpack_from(FRAME_PREFIX_FMT, payload, 0)
        points_bytes = payload[FRAME_PREFIX_LEN:]
        if len(points_bytes) != num_points * POINT_STEP:
            self.get_logger().warning(
                f"размер кадра не сходится: {num_points} точек, а данных {len(points_bytes)} байт — пропускаю")
            return

        if not self._first_frame_logged:
            self._first_frame_logged = True
            self.get_logger().info(f"первый кадр от UE5: {num_points} точек -> {self.lidar_topic}")

        with self._stats_lock:
            self._stats["received"] += 1
            self._stats["points"] += num_points
            self._stats["bytes"] += HEADER_LEN + len(payload)

        try:
            self._frame_queue.put_nowait((timestamp_sec, num_points, points_bytes))
        except queue.Full:
            # UE шлёт быстрее, чем публикуем — отбрасываем кадр: важна
            # свежесть, а не полнота (задержка иначе копилась бы).
            with self._stats_lock:
                self._stats["dropped"] += 1

    # ------------------------------------------------------------------ #
    # Публикация в ROS2 (только из потока executor'а)
    # ------------------------------------------------------------------ #
    def _publish_pending_frames(self):
        try:
            timestamp_sec, num_points, points_bytes = self._frame_queue.get_nowait()
        except queue.Empty:
            return

        header = Header()
        header.frame_id = self.frame_id
        if self.stamp_source == "ue":
            sec = int(timestamp_sec)
            header.stamp.sec = sec
            header.stamp.nanosec = int((timestamp_sec - sec) * 1e9)
        else:
            header.stamp = self.get_clock().now().to_msg()

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = num_points
        msg.fields = POINT_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * num_points
        msg.is_dense = True
        msg.data = array.array("B", points_bytes)
        self.lidar_pub.publish(msg)

        with self._stats_lock:
            self._stats["published"] += 1

    def _log_stats(self):
        with self._client_lock:
            addr = self._client_addr
        if addr is None:
            return
        with self._stats_lock:
            s, self._stats = self._stats, self._empty_stats()
        dt = max(1e-6, time.monotonic() - s["t0"])
        avg_pts = s["points"] / s["received"] if s["received"] else 0
        self.get_logger().info(
            f"UE5 -> ROS2: принято {s['received'] / dt:.1f} кадр/с, опубликовано "
            f"{s['published'] / dt:.1f} кадр/с, ~{avg_pts:.0f} точек/кадр, "
            f"{s['bytes'] / dt / 1e6:.2f} МБ/с, отброшено {s['dropped']}; "
            f"результатов отправлено в UE5: {s['results_sent']}")

    # ------------------------------------------------------------------ #
    # Подписки на результат их пайплайна -> пересылка в UE
    # ------------------------------------------------------------------ #
    def _on_safety_status(self, msg: String):
        with self._result_lock:
            self._status = STATUS_DANGER if msg.data.startswith("DANGER") else STATUS_SAFE
        self._send_result_to_ue()

    def _on_obstacle_distance(self, msg: Float32):
        with self._result_lock:
            self._distance = float(msg.data)
        self._send_result_to_ue()

    def _on_object_class(self, msg: String):
        # Формат от object_classifier_node: "id:name:confidence", напр. "4:obstacle:0.87"
        try:
            id_str, name, conf_str = msg.data.split(":", 2)
            with self._result_lock:
                self._class_id = int(id_str)
                self._class_name = name
                self._confidence = float(conf_str)
        except ValueError:
            self.get_logger().warning(f"не смог разобрать /metro/object_class: {msg.data!r}")
            return
        self._send_result_to_ue()

    def _send_result_to_ue(self):
        # Собираем пакет и кладём в очередь — без обращения к сокету.
        # Единственный sendall живёт в _sender_loop.
        with self._result_lock:
            status, distance = self._status, self._distance
            class_id, class_name, confidence = (
                self._class_id, self._class_name, self._confidence)

        name_bytes = class_name.encode("ascii", errors="replace")[:32].ljust(32, b"\0")
        payload = struct.pack(RESULT_FMT, status, distance, class_id, name_bytes, confidence)
        packet = struct.pack(HEADER_FMT, MAGIC, MSG_TYPE_ALGORITHM_RESULT, len(payload)) + payload

        try:
            self._out_queue.put_nowait(packet)
        except queue.Full:
            # Держим только самое свежее состояние; при гонке двух producer'ов
            # (executor + accept) возможно двойное выбрасывание — безвредно,
            # следующее состояние перепишет.
            try:
                self._out_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._out_queue.put_nowait(packet)
            except queue.Full:
                pass

    def _sender_loop(self):
        # Единственный поток, который блокируется на сокете. Executor'а не трогает.
        while rclpy.ok():
            try:
                packet = self._out_queue.get()
            except Exception:
                continue
            with self._client_lock:
                conn = self._client_sock
            if conn is None:
                continue  # обрыв: состояние теряется намеренно, свежее перепишет
            try:
                conn.sendall(packet)
            except OSError as e:
                self.get_logger().warning(f"не смог отправить результат в UE: {e}")
                continue
            # Статистика и лог — по факту отправленного байта, а не по
            # текущему self._status (между put и get состояние могло смениться).
            status, distance, *_ = struct.unpack(RESULT_FMT, packet[HEADER_LEN:])
            with self._stats_lock:
                self._stats["results_sent"] += 1
            if status != STATUS_UNKNOWN and not self._first_result_logged:
                self._first_result_logged = True
                self.get_logger().info(
                    f"первый результат пайплайна отправлен в UE5: "
                    f"{'DANGER' if status == STATUS_DANGER else 'SAFE'}, дистанция {distance:.2f} м")

    def close(self):
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass


def main():
    rclpy.init()
    node = UeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
