#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="metro_obstacle:dev"
DATA_DIR="$(cd "$SCRIPT_DIR/../data" 2>/dev/null && pwd || echo "$SCRIPT_DIR/../data")"

ensure_image() {
    if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
        echo "[run.sh] Образ не найден. Собираю..."
        docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
    fi
}

ROS_INIT="source /opt/ros/humble/setup.bash && source /ws/install/setup.bash"

run_pipeline() {
    local bag_path="$1"
    shift
    local duration=""
    local bag_args=()
    local remap=""
    
    # Автоматический remap для старых багов из for_hackathon
    if [[ "$bag_path" == *"for_hackathon"* ]]; then
        remap="--remap /sensing/lidar/hesai128/pointcloud:=/lidar_points"
    fi
    
    # Парсим --duration отдельно, остальные аргументы передаём в ros2 bag play
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --duration)
                duration="$2"
                shift 2
                ;;
            *)
                bag_args+=("$1")
                shift
                ;;
        esac
    done
    
    # Формируем команду воспроизведения
    local play_cmd="ros2 bag play /data/$bag_path $remap"
    if [[ ${#bag_args[@]} -gt 0 ]]; then
        play_cmd="$play_cmd ${bag_args[*]}"
    fi
    
    # Оборачиваем в timeout, если указана длительность
    if [[ -n "$duration" ]]; then
        play_cmd="timeout $duration $play_cmd"
    fi
    
    docker run --rm -it \
        --network host \
        --gpus all \
        -v "$DATA_DIR":/data \
        -v "$SCRIPT_DIR/scripts":/scripts:ro \
        "$IMAGE_NAME" \
        bash -lc "$ROS_INIT && { ros2 launch metro_obstacle_detection obstacle_detection.launch.py & } && sleep 3 && $play_cmd"
}

run_with_rviz() {
    local bag_cmd="$1"
    local extra_args="${2:-}"
    local remap=""
    
    # Автоматический remap для старых багов
    if [[ "$bag_cmd" == *"for_hackathon"* ]]; then
        remap="--remap /sensing/lidar/hesai128/pointcloud:=/lidar_points"
    fi
    
    docker run --rm -it \
        --network host --ipc=host --device /dev/dri \
        --gpus all \
        -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$DATA_DIR":/data \
        -v "$SCRIPT_DIR/scripts":/scripts:ro \
        "$IMAGE_NAME" \
        bash -lc "$ROS_INIT && { ros2 launch metro_obstacle_detection obstacle_detection.launch.py & } && sleep 2 && { rviz2 -d /ws/src/metro_obstacle_detection/config/rviz2_config.rviz & } && sleep 1 && $bag_cmd $remap $extra_args"
}

case "${1:-help}" in
    build)
        echo "[run.sh] Сборка Docker-образа..."
        docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
        echo "[run.sh] Готово."
        ;;

    run)
        BAG_PATH="${2:?Использование: ./run.sh run <bag_dir>}"
        shift 2
        
        if [[ "$BAG_PATH" == data/* ]]; then
            REL_PATH="${BAG_PATH#data/}"
        else
            REL_PATH="$BAG_PATH"
        fi
        
        ensure_image
        echo "[run.sh] Запуск на: $REL_PATH $*"
        run_pipeline "$REL_PATH" "$@"
        ;;

    test)
        ensure_image
        exec "$SCRIPT_DIR/test_all_bags.sh"
        ;;

    synthetic)
        shift
        ensure_image
        docker run --rm -it --network host --gpus all \
            -v "$DATA_DIR":/data -v "$SCRIPT_DIR/scripts":/scripts:ro \
            "$IMAGE_NAME" \
            bash -lc "$ROS_INIT && { ros2 launch metro_obstacle_detection obstacle_detection.launch.py & } && sleep 3 && python3 /scripts/synthetic_scene.py $*"
        ;;

    rviz)
        shift
        TARGET="${1:?Использование: ./run.sh rviz <bag_dir|synthetic>}"
        ensure_image
        if [ "$TARGET" = "synthetic" ]; then
            shift
            run_with_rviz "python3 /scripts/synthetic_scene.py $*" ""
        else
            if [[ "$TARGET" == data/* ]]; then
                REL_PATH="${TARGET#data/}"
            else
                REL_PATH="$TARGET"
            fi
            run_with_rviz "ros2 bag play /data/$REL_PATH" ""
        fi
        ;;

    help|--help|-h)
        echo "Использование: $0 {build|run|test|synthetic|rviz}"
        echo ""
        echo "Команды:"
        echo "  build                              Собрать Docker-образ"
        echo "  run <bag_dir> [опции]              Запуск на bag"
        echo "  test                               Прогнать все bag из data/for_hackathon/"
        echo "  synthetic [опции]                  Синтетическая сцена"
        echo "  rviz <bag|synthetic>               Запуск с RViz2"
        echo ""
        echo "Опции для run:"
        echo "  --duration SEC                     Ограничить время воспроизведения"
        echo "  --start-offset SEC                 Начать с указанной секунды"
        echo "  --rate FACTOR                      Скорость воспроизведения (1.0 = реальному времени)"
        echo ""
        echo "Примеры:"
        echo "  $0 build"
        echo "  $0 run data/new_data --duration 60"
        echo "  $0 run data/for_hackathon/doubleT_obstacle"
        echo "  $0 synthetic --distance 50"
        echo "  $0 rviz data/new_data"
        ;;

    *)
        echo "[run.sh] Неизвестная команда: $1"
        exit 1
        ;;
esac
