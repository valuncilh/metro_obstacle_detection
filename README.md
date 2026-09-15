# Обнаружение посторонних объектов в тоннеле метро

Проект команды DeepConv для хакатона ДепТранспорта.  
Разработка системы обнаружения препятствий перед беспилотным поездом метрополитена по данным 3D-лидара.

## Текущий статус

✅ Репозиторий инициализирован  
✅ Датасет распакован и проигрывается через Docker  
✅ Определены параметры лидара: Hesai, топик `/lidar_points`, 307200 точек/кадр, 10 Гц  
✅ Создан ROS 2 Python пакет `metro_obstacle_detection`  
✅ Написана базовая нода `lidar_listener`, подписывающаяся на облако точек  

🔲 В разработке: фильтрация по габариту, вычитание фона, детекция аномалий  
🔲 В планах: трекинг, оценка опасности, интеграция с системой принятия решений

## Окружение

- ОС: Ubuntu 22.04 LTS
- ROS 2: Humble
- Docker: обязателен для запуска
- Базовый образ: `ros:humble-ros-base`

> ⚠️ На хостовой системе ROS 2 не устанавливается. Вся разработка и запуск планируеться — через Docker.

## Структура проекта

```text
olimpiada_ltc/
├── data/
│   ├── raw/                  # Исходные архивы (не коммитятся)
│   └── for_hackathon/        # Распакованные бэги (не коммитятся)
├── metro_obstacle_detection/ # ROS 2 workspace
│   └── src/
│       └── metro_obstacle_detection/  # Python-пакет
│           ├── lidar_listener.py      # Нода подписки на /lidar_points
│           ├── setup.py
│           └── package.xml
├── inspect_lidar.py          # Вспомогательный скрипт инспекции данных
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

### 3. Запустить бэг

```bash
docker run --rm -it \
  --network host \
  -v $PWD/data:/data \
  ros:humble-ros-base \
  ros2 bag play /data/for_hackathon/doubleT_platform
```

### 4. Собрать и запустить ноду

```bash
docker run --rm -it \
  --network host \
  -v $PWD/metro_obstacle_detection:/ws \
  -w /ws \
  ros:humble-ros-base \
  bash -lc "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 run metro_obstacle_detection lidar_listener"
```

*(Запускайте в отдельном терминале параллельно с `ros2 bag play`)*

---

## Development Notes

### Внутренние соглашения

- **Ветки:** `main` — только стабильные версии, `dev` — интеграция, `feat/*` и `fix/*` — рабочие ветки.
- **Коммиты:** Conventional Commits (`feat:`, `fix:`, `docs:`, `exp:`, `chore:`).
- **Код:** Python — snake_case, ROS2-ноды — snake_case, топики — snake_case с префиксом `/metro/`.
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

### Локальная разработка (без Docker)

Если установлен ROS 2 Humble нативно (Ubuntu 22.04):

```bash
cd metro_obstacle_detection
colcon build --symlink-install
source install/setup.bash
ros2 run metro_obstacle_detection lidar_listener
```

### Отладка в Docker (try wsl)

Запуск оболочки контейнера с доступом к данным и коду:

```bash
docker run --rm -it \
  --network host \
  -v $PWD/data:/data \
  -v $PWD/metro_obstacle_detection:/ws \
  -w /ws \
  ros:humble-ros-base \
  bash -lc "source /opt/ros/humble/setup.bash && exec bash"
```

Внутри контейнера: сборка через `colcon build`, запуск нод через `ros2 run`.

### Чеклист перед пушем в dev

1. `colcon build` проходит без ошибок.
2. Нода запускается и подписывается на `/lidar_points`.
3. Нет закоммиченных данных, кэшей, `.pyc`, IDE-конфигов.
4. Обновлён `docs/experiments/log.md` (если менялся алгоритм).
4.1. Дополнен README.md с указанием добавденного/скоректированного функционала. 
5. `git status --short` чистый (кроме намеренных изменений).

## Контакты

- Мейнтейнер: vlaimir_vinogradov
- Почта: vv299907@xmail.ru
- Команда: DeepConv
