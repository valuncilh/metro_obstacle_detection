#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="metro_obstacle:dev"
DATA_DIR="$(cd "$SCRIPT_DIR/../data" && pwd)"
BAGS_DIR="$DATA_DIR/for_hackathon"
RESULTS="$SCRIPT_DIR/bench_results"
mkdir -p "$RESULTS"
TS="$(date +%Y%m%d_%H%M%S)"
CONTAINER="metro_bench_$TS"
STATS_CSV="$RESULTS/stats_$TS.csv"
PIPE_LOG="$RESULTS/pipeline_$TS.log"
DURATION="${BENCH_DURATION:-30}"

if [ "${1:-}" = "synthetic" ]; then
  shift
  SRC_CMD="python3 /scripts/synthetic_scene.py $*"
  LABEL="synthetic $*"
else
  BAG_NAME="${1:?Использование: ./benchmark.sh <bag_name|synthetic ...>}"
  if [ ! -d "$BAGS_DIR/$BAG_NAME" ]; then
    echo "Бэг не найден: $BAG_NAME"; exit 1
  fi
  SRC_CMD="ros2 bag play /data/for_hackathon/$BAG_NAME --remap /sensing/lidar/hesai128/pointcloud:=/lidar_points"
  LABEL="$BAG_NAME"
fi

echo "[bench] Сценарий: $LABEL | timeout: ${DURATION}s"

# --init убивает зомби-процессы при остановке контейнера
docker run -d --name "$CONTAINER" --init \
  --network host --gpus all \
  -v "$DATA_DIR":/data \
  -v "$SCRIPT_DIR/scripts":/scripts:ro \
  "$IMAGE_NAME" \
  bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
  ros2 launch metro_obstacle_detection obstacle_detection.launch.py & \
  sleep 3 && ros2 run metro_obstacle_detection metrics_logger & \
  sleep 1 && $SRC_CMD" > /dev/null

docker logs -f "$CONTAINER" > "$PIPE_LOG" 2>&1 &
LOG_PID=$!

echo "time_unix,cpu,mem_perc,mem_usage" > "$STATS_CSV"
START_T=$(date +%s)
while docker inspect "$CONTAINER" > /dev/null 2>&1; do
  ELAPSED=$(( $(date +%s) - START_T ))
  if [ "$ELAPSED" -ge "$DURATION" ]; then
    echo "[bench] Таймаут ${DURATION}s достигнут, останавливаю..."
    docker stop "$CONTAINER" > /dev/null 2>&1 || true
    break
  fi
  LINE="$(docker stats "$CONTAINER" --no-stream --format "{{.CPUPerc}},{{.MemPerc}},{{.MemUsage}}" 2>/dev/null || true)"
  echo "$(date +%s),$LINE" >> "$STATS_CSV"
  sleep 1
done

wait "$LOG_PID" 2>/dev/null || true
docker rm -f "$CONTAINER" > /dev/null 2>&1 || true

echo ""
echo "================ РЕЗУЛЬТАТЫ ================"
echo "Сценарий: $LABEL"
echo ""
echo "--- FPS и задержки ---"
grep "\[metrics\]" "$PIPE_LOG" | tail -10 || echo "(метрик нет)"
echo ""
echo "--- CPU / память ---"
awk -F"," 'NR>1 && NF>=4 {
  gsub(/%/,"",$2); gsub(/%/,"",$3);
  cpu=$2+0; mem=$3+0;
  s_cpu+=cpu; if(cpu>m_cpu)m_cpu=cpu;
  s_mem+=mem; if(mem>m_mem)m_mem=mem;
  n++; last=$4;
} END {
  if(n>0) printf "CPU: avg=%.1f%% max=%.1f%%\nMEM: avg=%.2f%% max=%.2f%% usage=%s\nЗамеров: %d\n", s_cpu/n, m_cpu, s_mem/n, m_mem, last, n;
}' "$STATS_CSV"
echo ""
echo "Файлы: $STATS_CSV | $PIPE_LOG"
