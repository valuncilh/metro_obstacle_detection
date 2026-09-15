#!/bin/bash
# Единая точка входа для системы обнаружения препятствий
# Использование:
#   ./run.sh build                  — собрать Docker-образ
#   ./run.sh <bag_name>             — запустить на реальном бэге
#   ./run.sh synthetic [опции]      — запустить на синтетической сцене
#   ./run.sh test                   — прогнать все бэги
#   ./run.sh rviz <bag_name>        — запустить с визуализацией RViz2
#
# Примеры:
#   ./run.sh doubleT_obstacle
#   ./run.sh synthetic --distance 50 --size 1.0
#   ./run.sh synthetic --distance 100 --size 0.5 --y 0.3
#   ./run.sh synthetic                           # пустой тоннель

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="metro_obstacle:dev"
DATA_DIR="$(cd "$SCRIPT_DIR/../data" && pwd)"
BAGS_DIR="$DATA_DIR/for_hackathon"

# Проверка наличия образа
ensure_image() {
    if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
        echo "[run.sh] Образ не найден. Собираю..."
        docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
    fi
}

# Запуск пайплайна + бэг в одном контейнере
run_pipeline() {
    local bag_path="$1"
    local remap="$2"
    docker run --rm -it \
        --network host \
        -v "$DATA_DIR":/data \
        -v "$SCRIPT_DIR/scripts":/scripts:ro \
        "$IMAGE_NAME" \
        bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
            ros2 launch metro_obstacle_detection obstacle_detection.launch.py & \
            sleep 3 && $bag_path $remap"
}

# Запуск с RViz2
run_with_rviz() {
    local bag_cmd="$1"
    local remap="$2"
    docker run --rm -it \
        --network host --ipc=host --device /dev/dri \
        -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$DATA_DIR":/data \
        -v "$SCRIPT_DIR/scripts":/scripts:ro \
        "$IMAGE_NAME" \
        bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
            ros2 launch metro_obstacle_detection obstacle_detection.launch.py & \
            sleep 2 && rviz2 -d /ws/src/metro_obstacle_detection/config/rviz2_config.rviz & \
            sleep 1 && $bag_cmd $remap"
}

case "${1:-help}" in
    build)
        echo "[run.sh] Сборка Docker-образа..."
        docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
        echo "[run.sh] Готово."
        ;;

    test)
        ensure_image
        exec "$SCRIPT_DIR/test_all_bags.sh"
        ;;

    synthetic)
        shift
        ensure_image
        echo "[run.sh] Синтетическая сцена: $*"
        docker run --rm -it \
            --network host \
            -v "$DATA_DIR":/data \
            -v "$SCRIPT_DIR/scripts":/scripts:ro \
            "$IMAGE_NAME" \
            bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
                ros2 launch metro_obstacle_detection obstacle_detection.launch.py & \
                sleep 3 && python3 /scripts/synthetic_scene.py $*"
        ;;

    rviz)
        shift
        BAG_NAME="${1:?Укажите имя бэга: ./run.sh rviz <bag_name>}"
        ensure_image
        if [ "$BAG_NAME" = "synthetic" ]; then
            shift
            run_with_rviz "python3 /scripts/synthetic_scene.py $*" ""
        else
            run_with_rviz "ros2 bag play /data/for_hackathon/$BAG_NAME" \
                "--remap /sensing/lidar/hesai128/pointcloud:=/lidar_points"
        fi
        ;;

    help|--help|-h)
        echo "Использование: $0 {build|test|synthetic|rviz|<bag_name>}"
        echo ""
        echo "Команды:"
        echo "  build                    Собрать Docker-образ"
        echo "  test                     Прогнать все бэги"
        echo "  synthetic [опции]        Синтетическая сцена"
        echo "  rviz <bag|synthetic>     Запуск с визуализацией"
        echo "  <bag_name>               Запуск на реальном бэге"
        echo ""
        echo "Опции synthetic:"
        echo "  --distance <м>           Дистанция до препятствия (по умолчанию: нет)"
        echo "  --size <м>               Размер препятствия (по умолчанию: 1.0)"
        echo "  --y <м>                  Смещение по Y (по умолчанию: 0.0)"
        echo "  --duration <с>           Длительность (по умолчанию: 10)"
        echo ""
        echo "Доступные бэги:"
        ls "$BAGS_DIR" 2>/dev/null || echo "  (нет данных)"
        ;;

    *)
        BAG_NAME="$1"
        BAG_PATH="$BAGS_DIR/$BAG_NAME"
        if [ ! -d "$BAG_PATH" ]; then
            echo "[run.sh] Ошибка: бэг не найден: $BAG_PATH"
            echo "Доступные бэги:"
            ls "$BAGS_DIR" 2>/dev/null
            exit 1
        fi
        ensure_image
        echo "[run.sh] Запуск на бэге: $BAG_NAME"
        run_pipeline "ros2 bag play /data/for_hackathon/$BAG_NAME" \
            "--remap /sensing/lidar/hesai128/pointcloud:=/lidar_points"
        ;;
esac
