# unitree_h1_control_ws
В данном репозитории лежат ROS2-пакеты, которые позволяют контролировать движения робота Unitree H1. Пакеты реализовывают как low-level управление, так и high-level управление.

## 📚 Оглавление

1. [📦 Содержание репозитория](#-содержание-репозитория)
2. [🚀 Быстрый старт](#-быстрый-старт)
3. [⚙️ Программные требования](#️-программные-требования)
4. [🧪 Использование](#-использование)
   - 4.1 [Запуск узлов](#запуск-узлов)
   - 4.2 [Запуск Launch файла](#запуск-launch-файла)
5. [📚 Кастомные библиотеки](#-кастомные-библиотеки)
   - 5.1 [`h1_info_library`](#h1_info_library)
6. [📡 Интерфейс (топики, сервисы, действия, параметры)](#-интерфейс-топики-сервисы-действия-параметры)
   - 6.1 [Пакет 1: `cmd_to_high_level_control_package`](#пакет-1-cmd_to_high_level_control_package)
   - 6.2 [Пакет 2: `high_level_control`](#пакет-2-high_level_control)
   - 6.3 [Пакет 3: `low_level_control`](#пакет-3-low_level_control)
   - 6.4 [Пакет 4: `gui_control`](#пакет-4-gui_control)
   - 6.5 [Пакет 5: `teleop_twist_keyboard_custom`](#пакет-5-teleop_twist_keyboard_custom)
7. [🌐 RQT-grahs's](#️-rqt-grahss)
8. [🧐 Лицензия](#-лицензия)
9. [😊 Благодарности](#-благодарности)
10. [💡 Предложения и корректировки](#-предложения-и-корректировки)

---

## 📦 Содержание репозитория
*   **`cmd_to_high_level_control_package/`**: Программа, анализирующая команды от **`teleop_twist_keyboard_custom`** и приводящая Unitree H1 в движение с помощью `high_level` команд.
* **`completed_scripts_control/`**: Содержит python launch-файл, который запускает конфигурации нод из данного репозитория.
*   **`docs/`**: Дополнительная документация.
*   **`gui_conrtol/`**: Позволяет задать положение каждому суставу робота с помощью графического интерфейса.
*   **`h1_info_library/`**: Python3-библиотека, содержит часто используемые в этом репозитории функции и словари, кастомную структуру данных.
*   **`high_level_control/`**: содержит high-level клиент, к
оторый позволяет управлять движением робота и изменять состояние робота, так же как с пульта.
*   **`low_level_control/`**: Модули, которые отправляют `low_level` команды для изменения положения моторов робота Unitree H1 и обеспечивают безопасность движений.
*   **`teleop_twist_keyboard/`**: Программа телеуправления (с помощью клавиатуры), адаптированная под Unitree H1.
*   **`unitree_sdk2_python/`**: Несколько переделанная библиотека от производителя робота, предназначенная для взаимодействия с роботом.
*   **`install_additions.bash`** - Bash-скрипт, который автоматизирует установку дополнительных зависимостей конкретно для данного репозитория.
*   **`install_dependensies.bash`** - Bash-скрипт, который автоматизирует установку python-зависимостей через pip.
*   **`README.md`**: Этот файл.
*   **`LICENSE`**: Лицензия, под которой распространяется данное программное обеспечение

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🚀 Быстрый старт

‼️**Репозиторий нужно устанавливать на роботе (Unitree H1), чтобы сохранить аппаратную привязку к устройствам.**

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
chmod +x install_additions.bash
sudo ./install_dependensies.bash
```
3. **Установить проприетарные зависимости** (по ссылкам инструкции по установке от производителя):
 - [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) По ссылке нужно выполнить всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду:
```bash
echo "source ~/unitree_ros2/cyclonedds_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4.  **Собрать workspace:**
```bash
cd $my_pwd/..
colcon build
source install/local_setup.bash  # Или local_setup.zsh
```
5.  Добавить `source` в `~/.bashrc`:
Зачем? - Чтобы не делать при каждом перезапуске терминала `source install/local_setup.bash`. Скрипт генерирует строчку для автоматизации этого процесса.
```bash
line_to_add="source \"$(pwd)/install/local_setup.bash\""
grep -qxF "$line_to_add" ~/.bashrc || echo "$line_to_add" >> ~/.bashrc
source ~/.bashrc
```
6. **Запустить пример / основной функционал:**
```bash
ros2 launch completed_scripts_control control_h1_base.launch.py
```

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>


## ⚙️ Программные требования

Всё, что нужно для шагов "Быстрого Старта" (устанавливается в "Быстром старте"):

*   **Поддерживаемые версии ROS2:** Foxy
*   **Поддерживаемые платформы:** Ubuntu 20.04
*   **Ключевые ROS2 пакеты:** `rclpy`, `std_msgs`, `geometry_msgs`, `unitree_go`
*   **Сторонние зависимости:** `numpy(pip)`, `serial(pip)`, `tkinter(pip)`, `rich(pip)`, `libboost-all-dev(apt)`, `libspdlog-dev(apt)`

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🧪 Использование
### **Запуск узлов:**
#### **`cmd_to_high_level_control_package`**
```bash
ros2 run cmd_to_high_level_control_package cmd_to_high_level_control_node
```
#### **`high_level_control`**
Минималистичный вывод в консоль:
```bash
ros2 run high_level_control simple_node
```
Красивый вывод в консоль:
```bash
ros2 run high_level_control beautiful_node
```
#### **`low_level_control`**
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
- `max_joint_velocity_param` - это максимальная скорость (рад/сек) для каждого из суставов. Значение по умолчанию = `4.0`
```bash
ros2 run low_level_control low_level_control_with_hands_node --ros-args -p target_topic_param:="lowcmd" -p max_joint_velocity_param:=1.5
```
#### **`gui_control`**
```bash
ros2 run gui_control gui_control_node
```
#### **`teleop_twist_keyboard_custom`**
```bash
ros2 run teleop_twist_keyboard_custom teleop_node
```

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>



### **Запуск Launch файла:**
#### Универсальный запуск с параметрами
```bash
ros2 launch completed_scripts_control control_h1_base.launch.py
```

**Доступные параметры запуска:**
- `mode` - режим работы: `with_hands` или `without_hands` (по умолчанию: `with_hands`)
- `target_topic` - топик для команд управления: `arm_sdk` или `lowcmd` (по умолчанию: `arm_sdk`)
- `target_action` - целевое действие: `other` или `teleoperation` (по умолчанию: `other`)
- `controled_by_cmd_vel` - управление через cmd_vel: `True` или `False` (по умолчанию: `False`)
- `max_joint_velocity` - максимальная скорость суставов (по умолчанию: `4.0`)
- `time_step` - временной шаг для управления движением (по умолчанию: `0.5`)

#### Примеры использования:

**Телеуправление с руками:**
```bash
ros2 launch completed_scripts_control control_h1_base.launch.py mode:=with_hands target_action:=teleoperation
```

**Телеуправление без рук:**
```bash
ros2 launch completed_scripts_control control_h1_base.launch.py mode:=without_hands target_action:=teleoperation
```

**SLAM с руками:**
```bash
ros2 launch completed_scripts_control control_h1_base.launch.py mode:=with_hands controled_by_cmd_vel:=True
```

**SLAM без рук:**
```bash
ros2 launch completed_scripts_control control_h1_base.launch.py mode:=without_hands controled_by_cmd_vel:=True
```

**Запускаемые ноды** (в зависимости от параметров, в формате `пакет` -> `зависимость`)::
- При `mode:=with_hands`:
  - `low_level_control` → `low_level_control_with_hands_node`
  - `low_level_control` → `wrist_control_node`
  - `low_level_control` → `hands_init_node`
- При `mode:=without_hands`:
  - `low_level_control` → `low_level_control_without_hands_node`
- При `controled_by_cmd_vel:=True`:
  - `cmd_to_high_level_control_package` → `cmd_to_high_level_control_node`

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

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
**`RobotData`** - класс, который содержит константы и оперативную информацию для low_level нод. При создании принимает 2 параметра (значения по умолчанию: `target_action = "teleoperation", include_hands_with_fingers = True`.) `target_action` позволяет выбрать задачу - это влияет на размеры рабочей области робота. `include_hands_with_fingers` - уточняет конфигурацию работа - в базовой комплектации или с установленными кистями Inspire Hands.

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
- `get_current_pose` - на вход получает абсолютный индекс сочленения. Метод возвращает информацию о действительном положении сочленения (float).
- `get_target_pose` - на вход получает абсолютный индекс сочленения. Метод возвращает информацию о целевом положении сочленения (float).
- `get_temporary_pose` - на вход получает абсолютный индекс сочленения. Метод возвращает информацию о промежуточном положении сочленения (float).

**`RobotState`** - класс, который представляет состояние всех моторов робота, включая основные суставы и пальцы (если подключены). При создании принимает 1 параметр (`include_hands_with_fingers = True`). `include_hands_with_fingers` определяет, нужно ли инициализировать моторы кистей (20-31 индекс).

**Поля**
1. `motors_info` - словарь (dict), где ключами являются абсолютные индексы моторов, а значениями - объекты структуры `CheckMotors`. Содержит справочную информацию о каждом моторе:
   - `abs_index`: int - абсолютный индекс мотора
   - `type`: str - тип мотора
   - `name_joint`: str - название сустава
   - `temperature`: int - текущая температура
   - `error`: int - сумма ошибки
   - `index_in_msg`: int - индекс в сообщении протокола

**Методы класса**
###### Основные методы доступа
- `get_motor` - принимает абсолютный индекс, возвращает объект `CheckMotors` для соответствующего мотора. Выбрасывает `ValueError`, если мотор не найден.
- `get_all_abs_indices` - возвращает список всех абсолютных индексов в порядке возрастания.
###### Методы получения свойств
- `get_joint_name_by_abs_index` - возвращает название сустава по абсолютному индексу
- `get_type_by_abs_index` - возвращает тип мотора по абсолютному индексу
- `get_error_by_abs_index` - возвращает код ошибки мотора по абсолютному индексу
- `get_temperature_by_abs_index` - возвращает температуру мотора по абсолютному индексу
- `get_msg_index_by_abs_index` - возвращает индекс мотора в сообщении протокола
###### Методы обновления состояния
- `update_temperature_by_abs_index` - обновляет температуру мотора по абсолютному индексу
- `update_error_by_abs_index` - обновляет код ошибки мотора по абсолютному индексу

##### `limits.py`
- `LIMITS_OF_JOINTS_WITH_HANDS_FROM_VENDOR ` - словарь: ключ - индекс сочленения робота Unitree H1, значение - пределы для этого сочленения от производителя
- `LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPRATION ` - словарь: ключ - индекс сочленения робота Unitree H1, значение - пределы для этого сочленения для телеуправления. Эти пределы урезаны по сравнению с пределами от производителя для того, чтобы рабочие области УКТ и Unitree H1 совпадали лучше.
##### `names_and_indexes.py`
- `FROM_NAMES_TO_INDEXES ` - словарь: ключ - имя сочленения Unitree H1, значение - индекс.
- `FROM_INDEXES_TO_NAMES ` - словарь: ключ - индекс Unitree H1, значение - название сочленения.
##### `names_and_numbers_of_messages.py`
- `FROM_NAMES_TO_NUMBERS_OF_MESSAGES ` - словарь: ключ - имя сочленения Unitree H1, значение - индекс-номер в соответствующем для данного типа сочленения сообщения.
##### `utils.py`
- `determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple` - функция, которая определяет коэффициенты Kp, Kd, а также mode для мотора в зависимости от его индекса. Для корректной работы необходимы следующие аргументы:
	- `index_of_joint_of_unitree_h1` - абсолютный индекс сочленения Unitree H1, для которого нужно узнать коэффициенты
	Возвращает картеж (Kp, Kd, mode), который соответствуют данному сочленению.

Нумерация суставов робота представлена ниже.
![unitree_h1_joints](docs/h1_with_hands.png)

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 📡 Интерфейс (топики, сервисы, действия, параметры)
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

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

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

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### **Пакет 3: `low_level_control`**
#### **Узел: `low_level_control_with_hands_node`**
- **Рабочие топики:**

| Тип услуги   | Топик                   | Тип сообщения                  | Описание                                                               |
| :----------- | :---------------------- | :----------------------------- | :--------------------------------------------------------------------- |
| Публикация   | `/inspire/cmd`          | `unitree_go/msg/MotorCmds`     | Целевые положения для пальцев-манипуляторов `Inspire Hands`            |
| Публикация   | `/wrist/cmds`<br>       | `unitree_go/msg/MotorCmds`<br> | Целевые положения для моторов `Damiao` в кистях<br>                    |
| Публикация   | `/{target_topic_param}` | `unitree_go/msg/LowCmd`        | Целевые положения для суставов `Unitree H1`                            |
| Подписка<br> | `/inspire/state`        | `unitree_go/msg/MotorStates`   | Степень расжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |
| Подписка<br> | `/wrist/states`         | `unitree_go/msg/MotorStates`   | Состояния суставов кистей `Unitree H1`                                 |
| Подписка<br> | `/lowstate`             | `unitree_go/msg/LowState`      | Состояния суставов робота `Unitree H1`                                 |
| Подписка     | `/positions_to_unitree` | `std_msgs/msg/String`          | Целевые положения для всех суставов робота в формате `JSON`            |

- **Параметры:**

| Параметр                   | Тип (знач. по умол.) | Описание                                                                     |
| :------------------------- | :------------------- | :--------------------------------------------------------------------------- |
| `max_joint_velocity_param` | `float (4.0)`        | Максимальная скорость для каждого из моторов, кроме пальцев                  |
| `target_topic_param`       | `string ('arm_sdk')`   | Имя топика, в который будут отправляться управляющие сообщения типа `LowCmd` |
| `target_action_param`    | `string ('teleoperation')`       | Целевое действие для задания рабочей области робота |
#### **Узел: `low_level_control_without_hands_node`**
- **Рабочие топики:**

| Тип услуги     | Топик                   | Тип сообщения             | Описание                                                    |
| :------------- | :---------------------- | :------------------------ | :---------------------------------------------------------- |
| Публикация     | `/{target_topic_param}` | `unitree_go/msg/LowCmd`   | Целевые значения положения для суставов `Unitree H1`        |
| Подписка<br>   | `/lowstate`             | `unitree_go/msg/LowState` | Состояния суставов робота `Unitree H1`                      |
| Подписка       | `/positions_to_unitree` | `std_msgs/msg/String`     | Целевые положения для всех суставов робота в формате `JSON` |

- **Параметры:**

| Параметр                   | Тип (знач. по умол.) | Описание                                                         |
| :------------------------- | :------------------- | :--------------------------------------------------------------- |
| `max_joint_velocity_param` | `float (4.0)`        | Максимальная скорость для всех моторов, кроме пальцев            |
| `target_topic_param`       | `string ('arm_sdk')`   | Имя топика, в который будут отправляться сообщения типа `LowCmd` |
| `target_action_param`    | `string ('teleoperation')`       | Целевое действие для задания рабочей области робота |

#### **Узел: `wrist_control_node`**
- **Рабочие топики:**

| Тип услуги     | Топик             | Тип сообщения                  | Описание                                            |
| :------------- | :---------------- | :----------------------------- | :-------------------------------------------------- |
| Публикация<br> | `/wrist/states`   | `unitree_go/msg/MotorStates`   | Состояния суставов кистей `Unitree H1`              |
| Подписка       | `/wrist/cmds`<br> | `unitree_go/msg/MotorCmds`<br> | Целевые положения для моторов `Damiao` в кистях<br> |

#### **Узел: `hands_init_node`**
- **Рабочие топики:** -
Узел сам топики не создаёт, но узел запускает собранный исполняемый файл, который запускает топики.
- Топики, с которыми взаимодействует запускаемая программа:

| Тип услуги   | Топик            | Тип сообщения                | Описание                                                               |
| :----------- | :--------------- | :--------------------------- | :--------------------------------------------------------------------- |
| Публикация   | `/inspire/cmd`   | `unitree_go/msg/MotorCmds`   | Целевые положения для пальцев-манипуляторов `Inspire Hands`            |
| Подписка<br> | `/inspire/state` | `unitree_go/msg/MotorStates` | Степень расжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>


### **Пакет 4: `gui_control`**
#### **Узел: `gui_control_node`**
- **Рабочие топики:**

| Тип услуги   | Топик                   | Тип сообщения                  | Описание                                                               |
| :----------- | :---------------------- | :----------------------------- | :--------------------------------------------------------------------- |
| Публикация   | `/positions_to_unitree` | `std_msgs/msg/String`          | Целевые положения для всех суставов робота в формате `JSON`            |
| Подписка<br> | `/inspire/state`        | `unitree_go/msg/MotorStates`   | Степень расжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |
<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

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

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🌐 RQT-grahs's
Здесь представлены скрины из rqt_graph при различных запущенных сценариях.
### `control_h1_base.launch.py mode:=with_hands`
![control_with_hands](docs/control_with_hands.png)
### `control_h1_base.launch.py mode:=without_hands`
![control_without_hands](docs/control_without_hands.png)
### `control_h1_base.launch.py mode:=with_hands controled_by_cmd_vel:=True`
![control_with_hands_with_cmd](docs/control_with_hands_with_cmd.png)
### `control_h1_base.launch.py mode:=without_hands controled_by_cmd_vel:=True`
![control_without_hands_with_cmd](docs/control_without_hands_with_cmd.png)

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>


## 🧐 Лицензия
Copyright (c) 2025 Алиса Зенина и Александр Грачев РТУ МИРЭА (Россия)

Данное программное обеспечение распространяется под [лицензией MIT](LICENSE).  
Разрешается свободное использование, копирование, модификация и распространение при условии сохранения уведомления об авторских правах и текста лицензии.

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 😊 Благодарности
Этот проект использует/модифицирует код из [репозитория Unitree Robotics SDK](https://github.com/unitreerobotics), который лицензирован в соответствии с **BSD 3-Clause License**.
Этот проект модифицирует код из [репозитория ros-teleop/twist_teleop_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard?ysclid=mebkz5791x655138144), который лицензирован в соответствии с **BSD License 2.0**.
Этот проект использует код из [репозитория dmBots/motor-control-routine](https://github.com/dmBots/motor-control-routine/tree/master/), который лицензирован в соответствии с **MIT**.
**Благодарим всех, кто косвенно участвовал в разработке!**

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 💡 Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>
