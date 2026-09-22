#!/bin/bash
set -e

MODE=${1:-gpu} # По умолчанию собираем с GPU
IMAGE_NAME="metro_obstacle:${MODE}"

echo "==> Сборка образа: $IMAGE_NAME (USE_CUDA=$( [ "$MODE" = "gpu" ] && echo "ON" || echo "OFF" ))"

docker build -t "$IMAGE_NAME" . \
  --build-arg USE_CUDA=$( [ "$MODE" = "gpu" ] && echo "ON" || echo "OFF" ) \
  --no-cache # Используем no-cache на хакатоне, чтобы гарантировать чистоту зависимостей

echo "==> Успешно собран образ: $IMAGE_NAME"
