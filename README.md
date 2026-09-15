# Обнаружение посторонних объектов в тоннеле метро

Проект команды DeepConv для хакатона ДепТранспорта.  
Разработка системы обнаружения препятствий перед беспилотным поездом метрополитена по данным 3D-лидара.

## Текущий статус

✅ Репозиторий инициализирован  
✅ Датасет распакован и проигрывается через Docker  
✅ Определены параметры лидара: Hesai, топик `/sensing/lidar/hesai128/pointcloud`, 307200 точек/кадр, 10 Гц  
✅ Создан ROS 2 C++ пакет `metro_obstacle_detection` с PCL  
✅ Реализован пайплайн: preprocessor → gauge_filter → anomaly_detector → decision_maker  
✅ Детекция препятствий работает на бэге `doubleT_obstacle` (2 кластера, статус DANGER)  
✅ Измерение дистанции до ближайшего препятствия  
✅ Публикация маркеров для визуализации в RViz2  

🔲 В разработке: вычитание фона, трекинг между кадрами, фильтрация ложных срабатываний  
🔲 В планах: оценка габаритов объекта, классификация опасности, интеграция с системой торможения

## Окружение

- ОС: любая с поддержкой Docker (Linux, Windows+WSL2, macOS)
- ROS 2: Humble
- C++17, PCL (Point Cloud Library)
- Docker: обязателен для запуска
- Базовый образ: `ros:humble-ros-base` + PCL зависимости

> На хостовой системе ROS 2 не требуется. Вся разработка и запуск — через Docker.

## Структура проекта

```text
olimpiada_ltc/
├── data/
│   ├── raw/                  # Исходные архивы (не коммитятся)
│   └── for_hackathon/        # Распакованные бэги (не коммитятся)
├── ros2_ws/                  # ROS 2 workspace
│   ├── Dockerfile            # Docker-образ с PCL и зависимостями
│   └── src/
│       └── metro_obstacle_detection/  # C++ пакет
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── src/
│           │   ├── preprocessor.cpp       # Фильтрация, прореживание
│           │   ├── gauge_filter.cpp       # Обрезка по габариту поезда
│           │   ├── anomaly_detector.cpp   # Кластеризация, детекция, дистанция
│           │   ├── decision_maker.cpp     # Принятие решения
│           │   └── debug_logger.cpp       # Отладочный логгер
│           ├── launch/
│           │   └── obstacle_detection.launch.py
│           └── config/
│               └── params.yaml
├── README.md
└── .gitignore
```

## Как начать работу (для команды)

### 1. Клонировать репозиторий

```bash
git clone <url_репозитория>
cd olimpiada_ltc
```

### 2. Получить датасет

Датасет не хранится в git. Скачайте архив `for_hackathon.zst` и положите его в `data/raw/`.

Распаковка:

```bash
tar --zstd -xvf data/raw/for_hackathon.zst -C data/
```

### 3. Собрать Docker-образ

```bash
docker build -t metro_obstacle:dev ros2_ws/
```

### 4. Запустить пайплайн с бэгом

```bash
docker run --rm -it \
  --network host \
  -v $PWD/data:/data \
  metro_obstacle:dev \
  bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
    ros2 launch metro_obstacle_detection obstacle_detection.launch.py & \
    sleep 3 && \
    ros2 bag play /data/for_hackathon/doubleT_obstacle \
      --remap /sensing/lidar/hesai128/pointcloud:=/lidar_points"
```

> **Важно:** реальный топик лидара в бэгах — `/sensing/lidar/hesai128/pointcloud`.  
> Необходимо использовать `--remap` для перенаправления на `/lidar_points`.

Доступные бэги для тестирования:
- `doubleT_platform` — пустой тоннель (ожидается `SAFE`)
- `doubleT_obstacle` — тоннель с препятствием (ожидается `DANGER`)
- `roundT_doubleT` — круглый тоннель, двойной путь
- `roundT_pressureGate_roundT` — круглый тоннель с гермозатвором
- `roundT_squareT_pressureGate_squareT` — комбинированный тоннель
- `squareT_platform_squareT_switch` — квадратный тоннель с платформой

---

## Топики системы

| Топик | Тип | Описание |
|-------|-----|----------|
| `/sensing/lidar/hesai128/pointcloud` | `sensor_msgs/msg/PointCloud2` | Входные данные лидара из бэга |
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | Вход пайплайна (после `--remap`) |
| `/metro/points_preprocessed` | `sensor_msgs/msg/PointCloud2` | После прореживания и фильтра дальности |
| `/metro/points_in_gauge` | `sensor_msgs/msg/PointCloud2` | После обрезки по габариту |
| `/metro/obstacles_raw` | `std_msgs/msg/String` | Результат детекции (статус + число кластеров) |
| `/metro/obstacle_distance` | `std_msgs/msg/Float32` | Дистанция до ближайшего препятствия (м) |
| `/metro/safety_status` | `std_msgs/msg/String` | Итоговый статус безопасности |
| `/metro/markers` | `visualization_msgs/msg/MarkerArray` | Маркеры для визуализации в RViz2 |

---

## Development Notes

### Внутренние соглашения

- **Ветки:** `main` — только стабильные версии, `dev` — интеграция, `feat/*` и `fix/*` — рабочие ветки.
- **Коммиты:** Conventional Commits (`feat:`, `fix:`, `docs:`, `exp:`, `chore:`).
- **Код:** топики — snake_case с префиксом `/metro/`.
- **Эксперименты:** Все результаты фиксируются в `docs/experiments/log.md` до коммита алгоритма.
- **Данные:** Никогда не коммитить файлы >10 МБ.

### Соглашения по C++

- Стандарт: C++17.
- Сборка: `ament_cmake`, `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- Стиль: близкий к ROS 2 / Google; форматирование через `clang-format` перед коммитом.
- Имена файлов: `snake_case.hpp`, `snake_case.cpp`.
- Классы и типы: `PascalCase`.
- Функции и переменные: `snake_case`.
- Поля классов: `snake_case_` с подчёркиванием на конце.
- Константы: `kPascalCase`.
- Ноды: параметры только через `declare_parameter` / `get_parameter`.
- Топики: `/metro/...`, если не требуется стандартный имя/формат.
- Колбэки не блокировать, тяжёлые вычисления выносить отдельно.
- Память: `std::unique_ptr` / `std::shared_ptr`, строго без ручного `new`/`delete`.

### Отладка в Docker

Запуск интерактивной оболочки контейнера с доступом к данным и коду:

```bash
docker run --rm -it \
  --network host \
  -v $PWD/data:/data \
  -v $PWD/ros2_ws/src:/ws/src \
  -w /ws \
  metro_obstacle:dev \
  bash -lc "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec bash"
```

Внутри контейнера:
- Сборка: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Запуск нод: `ros2 run metro_obstacle_detection <node_name>`
- Список топиков: `ros2 topic list`
- Просмотр топика: `ros2 topic echo /metro/safety_status`

### Чеклист перед пушем в dev

1. `docker build` проходит без ошибок.
2. Ноды запускаются и подписываются на `/lidar_points`.
3. Нет закоммиченных данных, кэшей, `.pyc`, IDE-конфигов.
4. Обновлён `docs/experiments/log.md` (если менялся алгоритм).
5. Дополнен `README.md` с указанием добавленного/скорректированного функционала.
6. `git status --short` чистый (кроме намеренных изменений).

## Контакты

- Мейнтейнер: vlaimir_vinogradov
- Почта: vv299907@xmail.ru
- Команда: DeepConv
