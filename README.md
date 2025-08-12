# unitree_h1_control_ws
В данном репозитории лежат ROS2-пакеты, которые позволяют контролировать движения робота Unitree H1. Пакеты реализовывают как low-level управление, так и high-level управление.

## 📦 Содержание Репозитория
*   **`cmd_to_high_level_control_package/`**: Программа, анализирующая команды от **`teleop_twist_keyboard_custom`** и приводящая Unitree H1 в движение с помощью `high_level` команд.
* **`completed_scripts_control/`**: Содержит python launch-файлы, которые запускают конфигурации нод из данного репозитория для задач соответствующих названиям launch-файлов.
*   **`docs/`**: Дополнительная документация (если есть).
*   **`gui_conrtol/`**: Позволяет задать положение каждому суставу робота с помощью графического интерфейса.
*   **`h1_info_library/`**: Python3-библиотека, содержит часто используемые в этом репозитории функции и словари, кастомную структуру данных.
*   **`high_level_control/`**: содержит high-level клиент, к
оторый позволяет управлять движением робота и изменять состояние робота, так же как с пульта.
* **`inspire_hands_lib/`**: библиотека для работы с `Inspire Hands`.
*   **`low_level_control_{}/`**: Модули, которые отправляют `low_level` команды для изменения положения моторов робота Unitree H1 и обеспечивают безопасность движений.
*   **`teleop_twist_keyboard/`**: Программа телеуправления (с помощью клавиатуры), адаптированная под Unitree H1.
*   **`unitree_sdk2_python/`**: Несколько переделанная библиотека от производителя робота, предназначенная для взаимодействия с роботом (содержит необходимые типы сообщений).
*   **`install_dependensies.bash`** - Bash-скрипт, который автоматизирует установку python-зависимостей через pip.
*   **`README.md`**: Этот файл.
*   **`save.bash`**: Скрипт для быстрой выгрузки на Github

## 🚀 Быстрый Старт
Пошаговая инструкция для **быстрого** запуска демо или основной функциональности. Предполагаем, что ROS2 (Foxy) уже установлен.

1.  **Клонировать репозиторий** в `src` вашего workspace:
```bash
mkdir -p unitree_h1_control_ws/src
cd unitree_h1_control_ws/src
git clone https://github.com/cyberbanana777/unitree_h1_control_ws.git .
my_pwd=$(pwd)
```
2. **Установить зависимости**:
```bash
sudo ./install_dependensies.bash
```
3. **Установить проприетарные зависимости** (по ссылкам инструкции по установке от производителя):
 - [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) По ссылке нужно выполнить всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду:
```bash
echo "source ~/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4.  **Собрать workspace:**
```bash
cd $my_pwd/..
colcon build
source install/setup.bash  # Или setup.zsh
```
5.  Добавить `source` в `~/.bashrc`:
Зачем? - Чтобы не делать при каждом перезапуске терминала `source install.setup.bash`. Скрипт генерирует строчку для автоматизации этого процесса.
```bash
line_to_add="source \"$(pwd)/install/setup.bash\""
grep -qxF "$line_to_add" ~/.bashrc || echo "$line_to_add" >> ~/.bashrc
source ~/.bashrc
```
6. **Запустить пример / основной функционал:**
```bash
ros2 launch completed_scripts_control control_H1_with_hands_launch.py
```
Или
```bash
ros2 launch completed_scripts_control control_H1_without_hands_launch.py
```
Или
```bash
ros2 launch completed_scripts_control control_for_slam_with_hands_launch.py
```
Или
```bash
ros2 launch completed_scripts_control control_for_slam_without_hands_launch.py
```
## ⚙️ Предварительные Требования

Всё, что нужно *до* шагов "Быстрого Старта":

*   **Поддерживаемые версии ROS2:** Foxy
*   **Поддерживаемые платформы:** Ubuntu 20.04
*   **Ключевые ROS2 пакеты:** `rclpy`, `std_msgs`, `geometry_msgs`, `unitree_go`
*   **Сторонние зависимости:** numpy(pip), serial(pip), tkinter(pip) rich(pip), libboost-all-dev(apt), libspdlog-dev(apt)

## 🧪 Использование
### **Запуск узлов:**
#### **cmd_to_high_level_control_package**
```bash
ros2 run cmd_to_high_level_control_package cmd_to_high_level_control_node
```
#### **high_level_control**
Минималистичный вывод в консоль:
```bash
ros2 run high_level_control simple_node
```
Красивый вывод в консоль:
```bash
ros2 run high_level_control beautiful_node
```
#### **low_level_control**
##### Без параметра
```bash
ros2 run low_level_control low_level_control_without_hands_node
```
или
```bash
ros2 run low_level_control low_level_control_with_hands_node
```
вместе с
```bash
ros2 run low_level_control wrist_control_node
```
и с
```bash
ros2 run low_level_control hands_init_node
```
##### С параметрами
Параметры:
- `target_topic_param` - это топик, куда будут отправляться управляющие сообщения для Unitree H1. Значение по умолчанию = `arm_sdk`
- `max_joint_velocity_param` - это максимальная скорость (рад/сек) для каждого из суставов. Значение по умолчанию = `7.0`
```bash
ros2 run low_level_control low_level_control_with_hands_node --ros-args -p target_topic_param:="lowcmd" -p max_joint_velocity_param:=1.5
```
#### **gui_control**
```bash
ros2 run gui_control gui_control_node
```
#### **teleop_twist_keyboard_custom**
```bash
ros2 run teleop_twist_keyboard_custom teleop_node
```

### **Запуск Launch файлов:**
#### Запуск телеуправления с руками
```bash
ros2 launch completed_scripts_control control_H1_with_hands_launch.py
```
**Запускаемые ноды** (в формате `пакет` -> `зависимость`):
- `low_level_control` -> `low_level_control_with_hands_node`
- `low_level_control` -> `wrist_control_node`
- `low_level_control` -> `hands_init_node`
#### Запуск для телеуправления без рук
```bash
ros2 launch completed_scripts_control control_H1_without_hands_launch.py
```
**Запускаемые ноды** (в формате `пакет` -> `зависимость`):
- `low_level_control` -> `low_level_control_without_hands_node`
#### Запуск для SLAM с руками
```bash
ros2 launch completed_scripts_control control_for_slam_with_hands_launch.py
```
**Запускаемые ноды** (в формате `пакет` -> `зависимость`):
- `low_level_control` -> `low_level_control_with_hands_node`
- `low_level_control` -> `wrist_control_node`
- `low_level_control` -> `hands_init_node`
- `cmd_to_high_level_control_package` -> `cmd_to_high_level_control_node`
#### Запуск для SLAM без рук
```bash
ros2 launch completed_scripts_control control_for_slam_without_hands_launch.py
```
**Запускаемые ноды** (в формате `пакет` -> `зависимость`):
- `low_level_control` -> `low_level_control_without_hands_node`
- `cmd_to_high_level_control_package` -> `cmd_to_high_level_control_node`

## 📚 Кастомные библиотеки
### **`h1_info_library`**
#### Структура пакета
```
h1_info_library
├── __init__.py
├── custom_dataclasses.py
├── limits.py
├── names_and_indexes.py
├── names_and_numbers_of_messages.py
└── utils.py
```
#### Объекты для взаимодействия
##### `custom_dataclasses.py`
- **`RobotData`** - класс, который содержит константы и оперативную информацию для low_level нод. При создании принимает 2 параметра (значения по умолчанию: `target_action = "teleoperation", include_hands_with_fingers = True`.) `target_action` позволяет выбрать задачу - это влияет на размеры рабочей области робота. `include_hands_with_fingers` - уточняет конфигурацию работа - в базовой комплектации или с установленными кистями Inspire Hands.
**Поля**
1. `joints_info` - list, объектами которого являются объекты кастомной подструктуры данных. Данное поле позволяет получить справочную информацию о сочленении(abs_index: int, type: str, name_joint: str, limits: tuple(float, float), index_in_msg: int ).
2. `joints_pose_status` - list, объектами которого являются объекты кастомной подструктуры данных. Данное поле хранит оперативную информацию для перевода моторов робота из одного положения в другое (abs_index: int, current_pose: float, temporary_pose: float, taget_pose: float)
**Методы класса**
- `show_info` - метод в основном для CLI-использования. При вызове - отрисовывает в красивом виде основную справочную информацию по суставам робота. Вызывается без аргументов.
- `get_joint_by_name` - на вход принимается имя сочленения, а возвращается объект кастомной структуры `joints_info` , со справочной информацией для данного сочленения.
- `get_joint_info_by_index` - на вход принимается абсолютный индекс, возвращает информацию  `joints_info`  о суставе с соответствующим абсолютным индексом. 
- `get_joint_pose_by_index` - на вход принимается абсолютный индекс, возвращает текущий объект `joints_pose_status` для сустава с соответствующим абсолютным индексом.
- `update_current_pose` - на вход получает абсолютный индекс сочленения и новое значение для поля `current_pose` (float) . Метод обновляет информацию о действительном положении сочленения.
- `update_target_pose` - на вход получает абсолютный индекс сочленения и новое значение для поля `target_pose` (float). Метод обновляет информацию о целевом положении сочленения.
- `update_temporary_pose` - на вход получает абсолютный индекс сочленения и новое значение для поля `temporary_pose` (float). Метод обновляет информацию о промежуточном положении сочленения.
##### `limits.py`
- **`LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR `** - словарь: ключ - индекс сочленения робота Unitree H1, значение - пределы для этого сочленения от производителя
- **`LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPRATION `** - словарь: ключ - индекс сочленения робота Unitree H1, значение - пределы для этого сочленения для телеуправления. Эти пределы урезаны по сравнению с пределами от производителя для того, чтобы рабочие области УКТ и Unitree H1 совпадали лучше.
##### `names_and_indexes.py`
- **`FROM_NAMES_TO_INDEXES `** - словарь: ключ - имя сочленения Unitree H1, значение - индекс.
- **`FROM_INDEXES_TO_NAMES `** - словарь: ключ - индекс Unitree H1, значение - название сочленения.
##### `names_and_numbers_of_messages.py`
- **`FROM_NAMES_TO_NUMBERS_OF_MESSAGES `** - словарь: ключ - имя сочленения Unitree H1, значение - индекс-номер в соответствующем для данного типа сочленения сообщения.
##### `utils.py`
- **`determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple`** - функция, которая определяет коэффициенты Kp, Kd, а также mode для мотора в зависимости от его индекса. Для корректной работы необходимы следующие аргументы:
	- `index_of_joint_of_unitree_h1` - абсолютный индекс сочленения Unitree H1, для которого нужно узнать коэффициенты
	Возвращает картеж (Kp, Kd, mode), который соответствуют данному сочленению.
## 📡 Интерфейс (Топики, Сервисы, Действия, Параметры)
Детальная спецификация API пакетов.
### **Пакет 1: `cmd_to_high_level_control_package`**
#### **Узел: `cmd_to_high_level_control_node`**
- **Рабочие топики:**

| Тип услуги | Топик      | Тип сообщения             | Описание                                 |
| :--------- | :--------- | :------------------------ | :--------------------------------------- |
| Публикация | `/cmd_vel` | `geometry_msgs/msg/Twist` | Целевые скорости (линейные и ангулярные) |
| Подписка   | `/cmd_vel` | `geometry_msgs/msg/Twist` | Целевые скорости (линейные и ангулярные) |

- **Параметры:**

| Параметр    | Тип (знач. по умол.) | Описание                                      |
| :---------- | :------------------- | :-------------------------------------------- |
| `time_step` | `float (0.5)`        | Длительность движения при однократном нажатии |

### **Пакет 2: `high_level_control`**
#### **Узел: `high_level_control_simple_node`**
- **Рабочие топики:**

| Тип услуги | Топик                | Тип сообщения              | Описание                                         |
| :--------- | :------------------- | :------------------------- | :----------------------------------------------- |
| Публикация | `/api/loco/request`  | `unitree_api/msg/Request`  | Запрос на выполнение high-level-команды          |
| Подписка   | `/api/loco/response` | `unitree_api/msg/Response` | Ответ на запрос о  выполнении high-level-команды |
#### **Узел: `high_level_control_beautiful_node`**
- **Рабочие топики:**

| Тип услуги | Топик                | Тип сообщения              | Описание                                         |
| :--------- | :------------------- | :------------------------- | :----------------------------------------------- |
| Публикация | `/api/loco/request`  | `unitree_api/msg/Request`  | Запрос на выполнение high-level-команды          |
| Подписка   | `/api/loco/response` | `unitree_api/msg/Response` | Ответ на запрос о  выполнении high-level-команды |

### **Пакет 3: `low_level_control`**
#### **Узел: `low_level_control_with_hands_node`**
- **Рабочие топики:**

| Тип услуги     | Топик                   | Тип сообщения                  | Описание                                                               |
| :------------- | :---------------------- | :----------------------------- | :--------------------------------------------------------------------- |
| Публикация     | `/inspire/cmd`          | `unitree_go/msg/MotorCmds`     | Целевые положения для пальцев-манипуляторов `Inspire Hands`            |
| Публикация     | `/wrist/cmds`<br>       | `unitree_go/msg/MotorCmds`<br> | Целевые положения для моторов `Damiao` в кистях<br>                    |
| Публикация     | `/{target_topic_param}` | `unitree_go/msg/LowCmd`        | Целевые положения для суставов `Unitree H1`                            |
| Публикация<br> | `/max_temperature`      | `std_msgs/msg/Float`           | Максимальная температура среди всех моторов в системе                  |
| Подписка<br>   | `/inspire/state`        | `unitree_go/msg/MotorStates`   | Степень расжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |
| Подписка<br>   | `/wrist/states`         | `unitree_go/msg/MotorStates`   | Состояния суставов кистей `Unitree H1`                                 |
| Подписка<br>   | `/lowstate`             | `unitree_go/msg/LowState`      | Состояния суставов робота `Unitree H1`                                 |
| Подписка       | `/positions_to_unitree` | `std_msgs/msg/String`          | Целевые положения для всех суставов робота в формате `JSON`            |

- **Параметры:**

| Параметр                   | Тип (знач. по умол.) | Описание                                                                     |
| :------------------------- | :------------------- | :--------------------------------------------------------------------------- |
| `max_joint_velocity_param` | `float (7.0)`        | Максимальная скорость для каждого из моторов, кроме пальцев                  |
| `target_topic_param`       | `string (arm_sdk)`   | Имя топика, в который будут отправляться управляющие сообщения типа `LowCmd` |

#### **Узел: `low_level_control_without_hands_node`**
- **Рабочие топики:**

| Тип услуги     | Топик                   | Тип сообщения             | Описание                                                    |
| :------------- | :---------------------- | :------------------------ | :---------------------------------------------------------- |
| Публикация     | `/{target_topic_param}` | `unitree_go/msg/LowCmd`   | Целевые значения положения для суставов `Unitree H1`        |
| Публикация<br> | `/max_temperature`      | `std_msgs/msg/Float`      | Максимальная температура среди всех моторов в системе       |
| Подписка<br>   | `/lowstate`             | `unitree_go/msg/LowState` | Состояния суставов робота `Unitree H1`                      |
| Подписка       | `/positions_to_unitree` | `std_msgs/msg/String`     | Целевые положения для всех суставов робота в формате `JSON` |

- **Параметры:**

| Параметр                   | Тип (знач. по умол.) | Описание                                                         |
| :------------------------- | :------------------- | :--------------------------------------------------------------- |
| `max_joint_velocity_param` | `float (7.0)`        | Максимальная скорость для всех моторов, кроме пальцев            |
| `target_topic_param`       | `string (arm_sdk)`   | Имя топика, в который будут отправляться сообщения типа `LowCmd` |

#### **Узел: `wrist_control_node`**
- **Рабочие топики:**

| Тип услуги     | Топик             | Тип сообщения                  | Описание                                            |
| :------------- | :---------------- | :----------------------------- | :-------------------------------------------------- |
| Публикация<br> | `/wrist/states`   | `unitree_go/msg/MotorStates`   | Состояния суставов кистей `Unitree H1`              |
| Подписка       | `/wrist/cmds`<br> | `unitree_go/msg/MotorCmds`<br> | Целевые положения для моторов `Damiao` в кистях<br> |

#### **Узел: `hands_init_node`**
- **Рабочие топики:** -

### **Пакет 4: `gui_control`**
#### **Узел: `gui_control_node`**
- **Рабочие топики:**

| Тип услуги | Топик                   | Тип сообщения         | Описание                                                    |
| :--------- | :---------------------- | :-------------------- | :---------------------------------------------------------- |
| Публикация | `/positions_to_unitree` | `std_msgs/msg/String` | Целевые положения для всех суставов робота в формате `JSON` |


### **Пакет 5: `teleop_twist_keyboard_custom`**
#### **Узел: `teleop_twist_keyboard_custom`**
- **Рабочие топики:**

| Тип услуги | Топик      | Тип сообщения             | Описание                                 |
| :--------- | :--------- | :------------------------ | :--------------------------------------- |
| Публикация | `/cmd_vel` | `geometry_msgs/msg/Twist` | Целевые скорости (линейные и ангулярные) |

- **Параметры:**

| Параметр   | Тип (знач. по умол.) | Описание                              |
| :--------- | :------------------- | :------------------------------------ |
| `stamped`  | `Bool (False)`       | Добавление штампа времени в сообщение |
| `frame_id` | `String ('')`        | Система координат (frame)             |

## 🗺️ Архитектура
RQT-графы будут представлены тут.

## Благодарности
Этот проект использует/модифицирует код из [репозитория Unitree Robotics SDK](https://github.com/unitreerobotics), который лицензирован в соответствии с **BSD 3-Clause License**.
Этот проект включает код из [репозитория inspire_hands](https://github.com/Sentdex/inspire_hands/tree/main), лицензированный под MIT.
## Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).