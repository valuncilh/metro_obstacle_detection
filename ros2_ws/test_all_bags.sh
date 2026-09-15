#!/bin/bash
# Тестирование пайплайна на всех бэгах датасета
# Использование: ./test_all_bags.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="$(cd "$SCRIPT_DIR/../data" && pwd)"
BAGS_DIR="$DATA_DIR/for_hackathon"
RESULTS_DIR="$SCRIPT_DIR/test_results"

mkdir -p "$RESULTS_DIR"

echo "=== Тестирование пайплайна на всех бэгах ==="
echo ""

for BAG_PATH in "$BAGS_DIR"/*/; do
    BAG_NAME=$(basename "$BAG_PATH")
    RESULT_FILE="$RESULTS_DIR/$BAG_NAME.log"
    
    echo "--- Тест: $BAG_NAME ---"
    
    # Запускаем пайплайн на 15 секунд и сохраняем вывод
    timeout 15 docker run --rm \
        --network host \
        -v "$DATA_DIR":/data \
        metro_obstacle:dev \
        bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
            ros2 launch metro_obstacle_detection obstacle_detection.launch.py & \
            sleep 2 && \
            ros2 bag play /data/for_hackathon/$BAG_NAME --remap /sensing/lidar/hesai128/pointcloud:=/lidar_points" \
        > "$RESULT_FILE" 2>&1 || true
    
    # Извлекаем статистику
    TOTAL_FRAMES=$(grep -c "Detection:" "$RESULT_FILE" || echo "0")
    OBSTACLE_FRAMES=$(grep -c "obstacle_detected" "$RESULT_FILE" || echo "0")
    SAFE_FRAMES=$(grep -c "no_obstacle" "$RESULT_FILE" || echo "0")
    
    # Дистанции
    DISTANCES=$(grep "distance:" "$RESULT_FILE" | grep -oP "distance: \d+" | grep -oP "\d+" | sort -n | uniq -c || echo "нет")
    
    echo "  Всего кадров: $TOTAL_FRAMES"
    echo "  С препятствием: $OBSTACLE_FRAMES"
    echo "  Без препятствия: $SAFE_FRAMES"
    echo "  Дистанции (кол-во × метр):"
    echo "$DISTANCES" | sed "s/^/    /"
    echo ""
done

echo "=== Полные логи сохранены в: $RESULTS_DIR ==="
