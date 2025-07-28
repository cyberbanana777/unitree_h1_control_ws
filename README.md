@ - отложено до того момента как программа не будет полностью дописана

# @low_level_control
## Осталось
- Проверить код обоих нод по pep8
- Проверить аннотацию в python-кодах
- Сделать описание в setup.py
- Сделать описание в package.xml
## Данное состояние
- Нода работает, но не описана

# @gui_control
## Осталось
- Сделать так, чтобы отправляла за 1 раз и хранила значения всех суставов
## Данное состояние
- Нода работает, но не совсем корректно (возможны улучшения)


# unitree_h1_control_ws
В данном репозитории лежат ROS2-пакеты, которые позволяют контролировать движения робота Unitree H1. Пакеты реализовывают как low-level движение, так и high-level движение.

## 📦 Содержание Репозитория
*   **`cmd_to_high_level_control_package/`**: Программа, анализирующая команды от **`teleop_twist_keyboard_custom`** и приводящая Unitree H1 в движение с помощью `high_level` команд.
* **`completed_scripts_control/`**: Содержит python launch-файлы, которые запускают конфигурации нод из данного репозитория для задач соответствующих названиям launch-файлов.
*   **`high_level_control/`**: содержит high-level клиент, который позволяет управлять движением робота и изменять состояние робота, так же как с пульта.
*   **`low_level_control/`**: Модули, которые отправляют `low_level` команды для изменения положения моторов и робота Unitree H1 и обеспечивают безопасность движений робота.
*   **`gui_conrtol/`**: Позволяет задать положение робота  с помощью графического интерфейса.
*   **`h1_info_library/`**: Библиотека, разработанная нами. В ней содержится справочная информация для работы с Unitree H1 и кастомная структура информации (класс ООП), которые облегчает взаимодействие с роботом.
*   **`teleop_twist_keyboard/`**: Программа телеуправления, адаптированная под Unitree H1.
*   **`unitree_sdk2_python/`**: Немного переделанная библиотека от производителя робота, предназначенная для взаимодействия с роботом.
*   **`docs/`**: Дополнительная документация (если есть).
*   **`install_dependensies.bash`** - Bash-скрипт, который автоматизирует установку python-зависимостей через pip.
*   **`README.md`**: Этот файл.
*   **`save.bash`**: Скрипт для быстрой выгрузки на Github

## 🚀 Быстрый Старт
Пошаговая инструкция для **быстрого** запуска демо или основной функциональности. Предполагаем, что ROS2 уже установлен.

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
*   **Поддерживаемые платформы:** Ubuntu 22.04
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
ros2 run low_level_control low_level_control_with_hands_node
```
или
```bash
ros2 run low_level_control low_level_control_withouthands_node
```
или
```bash
ros2 run low_level_control wrist_control_node
```
или
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
ros2 run teleop_twist_keyboard_custom teleop_twist_keyboard_custom_node
```

### **Запуск Launch файлов:**
#### Запуск для телеуправления с руками
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

| Тип услуги | Топик                | Тип сообщения              | Описание                                      |
| :--------- | :------------------- | :------------------------- | :-------------------------------------------- |
| Публикация | `/api/loco/request`  | `unitree_api/msg/Request`  | Запрос на выполнение high-level-команды       |
| Подписка   | `/api/loco/response` | `unitree_api/msg/Response` | Ответ на запрос выполнения high-level-команды |
#### **Узел: `high_level_control_beautiful_node`**
- **Рабочие топики:**

| Тип услуги | Топик                | Тип сообщения              | Описание                                      |
| :--------- | :------------------- | :------------------------- | :-------------------------------------------- |
| Публикация | `/api/loco/request`  | `unitree_api/msg/Request`  | Запрос на выполнение high-level-команды       |
| Подписка   | `/api/loco/response` | `unitree_api/msg/Response` | Ответ на запрос выполнения high-level-команды |

### **Пакет 3: `low_level_control`**
#### **Узел: `low_level_control_with_hands_node`**
- **Рабочие топики:**

| Тип услуги     | Топик                   | Тип сообщения                  | Описание                                                               |
| :------------- | :---------------------- | :----------------------------- | :--------------------------------------------------------------------- |
| Публикация     | `/inspire/cmd`          | `unitree_go/msg/MotorCmds`     | Целевые значения для пальцев-манипуляторов `Inspire Hands`             |
| Публикация     | `/wrist/cmds`<br>       | `unitree_go/msg/MotorCmds`<br> | Целевые значения для моторов `Damiao` в кистях<br>                     |
| Публикация     | `/{target_topic_param}` | `unitree_go/msg/LowCmd`        | Целевые значения для моторов `Unitree H1`                              |
| Публикация<br> | `/max_temperature`      | `std_msgs/msg/Float`           | Максимальная температура среди всех моторов в системе                  |
| Подписка<br>   | `/inspire/state`        | `unitree_go/msg/MotorStates`   | Степень расжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |
| Подписка<br>   | `/wrist/states`         | `unitree_go/msg/MotorStates`   | Состояния суставов кистей `Unitree H1`                                 |
| Подписка<br>   | `/lowstate`             | `unitree_go/msg/LowState`      | Состояния суставов робота `Unitree H1`                                 |
| Подписка       | `/positions_to_unitree` | `std_msgs/msg/String`          | Целевые значения для всех суставов робота в формате `JSON`             |

- **Параметры:**

| Параметр                   | Тип (знач. по умол.) | Описание                                                                              |
| :------------------------- | :------------------- | :------------------------------------------------------------------------------------ |
| `max_joint_velocity_param` | `float (7.0)`        | Максимальная скорость для каждого из суставов                                         |
| `target_topic_param`       | `string (arm_sdk)`   | Имя топика, в который будут отправляться управляющие моторами сообщения типа `LowCmd` |

#### **Узел: `low_level_control_without_hands_node`**
- **Рабочие топики:**

| Тип услуги     | Топик                   | Тип сообщения             | Описание                                              |
| :------------- | :---------------------- | :------------------------ | :---------------------------------------------------- |
| Публикация     | `/{target_topic_param}` | `unitree_go/msg/LowCmd`   | Целевые значения для моторов `Unitree H1`             |
| Публикация<br> | `/max_temperature`      | `std_msgs/msg/Float`      | Максимальная температура среди всех моторов в системе |
| Подписка<br>   | `/lowstate`             | `unitree_go/msg/LowState` | Состояния суставов робота `Unitree H1`                |
| Подписка       | `/positions_to_unitree` | `std_msgs/msg/String`     | Целевые значения всего робота в формате `JSON`        |

- **Параметры:**

| Параметр                   | Тип (знач. по умол.) | Описание                                                         |
| :------------------------- | :------------------- | :--------------------------------------------------------------- |
| `max_joint_velocity_param` | `float (7.0)`        | Максимальная скорость для всех суставов                          |
| `target_topic_param`       | `string (arm_sdk)`   | Имя топика, в который будут отправляться сообщения типа `LowCmd` |

#### **Узел: `wrist_control_node`**
- **Рабочие топики:**

| Тип услуги     | Топик             | Тип сообщения                  | Описание                                           |
| :------------- | :---------------- | :----------------------------- | :------------------------------------------------- |
| Публикация<br> | `/wrist/states`   | `unitree_go/msg/MotorStates`   | Состояния суставов кистей `Unitree H1`             |
| Подписка       | `/wrist/cmds`<br> | `unitree_go/msg/MotorCmds`<br> | Целевые значения для моторов `Damiao` в кистях<br> |

#### **Узел: `hands_init_node`**
- **Рабочие топики:** -

### **Пакет 4: `gui_control`**
#### **Узел: `gui_control_node`**
- **Рабочие топики:**

| Тип услуги | Топик                   | Тип сообщения         | Описание                                                    |
| :--------- | :---------------------- | :-------------------- | :---------------------------------------------------------- |
| Публикация | `/positions_to_unitree` | `std_msgs/msg/String` | Целевые значения для всех ссуставов робота в формате `JSON` |


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
Этот проект использует/модифицирует код из [Unitree Robotics SDK](https://github.com/unitreerobotics), который лицензирован в соответствии с **BSD 3-Clause License**.
## Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).
