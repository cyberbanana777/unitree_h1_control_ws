@ - отложено до того момента как программа не будет полностью дописана
# @cmd_to_high_level_control_package
## Осталось
- Сделать аннотацию в python-коде
- Сделать описание в setup.py
- Сделать описание в package.xml
## Данное состояние
- Нода работает, но не описана

# @low_level_control
## Осталось
- Проверить код обоих нод по pep8
- Проверить аннотацию в python-кодах
- Сделать описание в setup.py
- Сделать описание в package.xml
## Данное состояние
- Нода работает, но не описана

# @slider_control
## Осталось
- Проверить код по pep8
- Сделать аннотацию в python-коде
- Сделать описание в setup.py
- Сделать описание в package.xml
## Данное состояние
- Нода работает, но не описана


# unitree_h1_control_ws
В данном репозитории лежат ROS2-пакеты, которые позволяют контролировать движения робота Unitree H1. Пакеты реализовывают как low-level движение, так и high-level движение. 

## 📦 Содержание Репозитория
*   **`cmd_to_high_level_control_package/`**: Вторая программа, анализирующая команды из **`teleop_twist_keyboard_custom`** и приводящая Unitree H1 в необходимое движение.
* **`completed_scripts_control/`**: Содержит python launch-файлы, которые запускают конфигурации нод из данного репозитория для задач соответствующих названиям launch-файлов.
*   **`high_level_control/`**: содержит high-level клиент, который позволяет управлять движением робота и изменять состояние робота .
*   **`low_level_control/`**: Модули, которые отправляют непосредственное воздействие прямиком на моторы робота Unitree H1 и отвечает за безопасность движений робота как для окружающих, так и для себя самого.
*   **`slider_conrtol/`**: Позволяет задать положение робота  с помощью графического интерфейса.
*   **`teleop_twist_keyboard/`**: Программа телеуправления, адаптированная под Unitree H1.
*   **`unitree_sdk2_python/`**: Немного переделанная библиотека от производителя робота, предназначенная для взаимодействия с роботом.
*   ==**`docs/`**: Дополнительная документация (если есть)==.
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
 - [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) Здесь мы выполняем всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду:
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

Детализируйте *все*, что нужно *до* шагов "Быстрого Старта":

*   **Поддерживаемые версии ROS2:** Foxy
*   **Поддерживаемые платформы:** Ubuntu 22.04
*   **Ключевые ROS2 пакеты:** `rclpy`, `std_msgs`, `geometry_msgs`, `unitree_go`
*   **Сторонние зависимости:** numpy(pip), serial(pip), tkinter(pip), libboost-all-dev(apt), libspdlog-dev(apt)

## 🧪 Использование
### **Запуск узлов:**
#### cmd_to_high_level_control_package
```bash 
ros2 run cmd_to_high_level_control_package cmd_to_high_level_control_node
```
#### high_level_control
```bash
ros2 run high_level_control high_level_control_node
```
#### low_level_control
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
- `max_joint_velocity_param` - это максимальная скорость (в рад/сек) для каждого из суставов. Значение по умолчанию = `7.0`
```bash
ros2 run low_level_control low_level_control_with_hands_node --ros-args -p target_topic_param:="lowcmd" -p max_joint_velocity_param:=1.5 
```
#### slider_control
```bash
ros2 run slider_control slider_control_node
```
#### teleop_twist_keyboard_custom
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
Детальная спецификация API пакетов. Обычно таблицы.
### **Пакет 1: cmd_to_high_level_control_package**
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

### **Пакет 2: high_level_control**
#### **Узел: `high_level_control_node`**
- **Рабочие топики:**

| Тип услуги | Топик                | Тип сообщения              | Описание                                                                                |
| :--------- | :------------------- | :------------------------- | :-------------------------------------------------------------------------------------- |
| Публикация | `/api/loco/request`  | `unitree_api/msg/Request`  | Целевые угловые положения для суставов Unitree H1 в допустимом диапазоне для Unitree H1 |
| Подписка   | `/api/loco/response` | `unitree_api/msg/Response` | Исходные угловые положения суставов с УКТ в JSON-формате                                |

### **Пакет 3: extractor_package**
#### **Узел: `extractor_node `**
- **Рабочие топики:**

| Тип услуги   | Топик                                       | Тип сообщения                | Описание                                                                                                                  |
| :----------- | :------------------------------------------ | :--------------------------- | :------------------------------------------------------------------------------------------------------------------------ |
| Публикация   | `/plotjuggler/joint_{H1_joint_num}/H1`      | `std_msgs/msg/Float32`       | Угловое положение выбранного сустава УКТ  в радианах или степень разжатия пальца в условных единицах от 0 до 1            |
| Публикация   | `/plotjuggler/joint_{H1_joint_num}/UKT`<br> | `std_msgs/msg/Float32`<br>   | Угловое положение выбранного сустава Unitree H1  в радианах или степень разжатия пальца в условных единицах от 0 до 1<br> |
| Подписка     | `/UKT_data_rad`                             | `std_msgs/msg/String`        | Угловые положения суставов  УКТ в радианах и степень разжатия пальца в условных единицах от 0 до 1                        |
| Подписка<br> | `/inspire/state`                            | `unitree_go/msg/MotorStates` | Степень разжатия пальцев `Inspire Hands` в условных единицах от 0 до 1                                                    |
| Подписка<br> | `/lowstate`                                 | `unitree_go/msg/LowState`    | Состояния суставов робота `Unitree H1`                                                                                    |

- **Параметры:**

| Параметр       | Тип (знач. по умол.) | Описание                                                                                     |
| :------------- | :------------------- | :------------------------------------------------------------------------------------------- |
| `H1_joint_num` | `int (16)`           | Номер сустава, угловое положение которого отслеживается. Нельзя изменить после запуска ноды. |


## 🗺️ Архитектура
RQT-графы будут представлены тут.

## Credits
This project uses/modifies code from the [Unitree Robotics SDK](https://github.com/unitreerobotics), which is licensed under the **BSD 3-Clause License**.

## Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).