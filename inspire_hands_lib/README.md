# Inspire Hand Python Library

A Python library for controlling the Inspire Hand RH56dfq robotic hand through a serial connection using Modbus RTU protocol.

## Features

- Complete control of the Inspire Hand RH56dfq
- Pythonic API for intuitive hand control
- Individual finger manipulation
- Pre-defined gestures (pinch, point, thumbs-up, grip)
- Status monitoring and error handling
- Support for speed and force control

## Installation

```bash
pip install git+https://github.com/yourusername/inspire-hand-pkg.git
```

## Quick Start

To just quickly interact with the CLI manually (CLI can also be called purely from terminal/bash commands, see below).

```bash
python -m inspire_hand.cli interactive
```

Interacting within a .py script:

```python
from inspire_hand import InspireHand

# Connect to the hand
with InspireHand(port='/dev/ttyUSB0') as hand:
    # Open all fingers
    hand.open_all_fingers()
    
    # Close just the index finger
    hand.index_finger.close()
    
    # Make a pinch gesture
    hand.pinch(force=500)
    
    # Get the current angle of the thumb
    thumb_angle = hand.thumb_bend.angle
    print(f"Thumb angle: {thumb_angle}")
```

## Documentation

### Connecting to the Hand

```python
from inspire_hand import InspireHand

# Option 1: Using context manager (recommended)
with InspireHand(port='/dev/ttyUSB0', baudrate=115200) as hand:
    # Your code here
    pass

# Option 2: Manual connection
hand = InspireHand(port='/dev/ttyUSB0', baudrate=115200)
hand.open()
# Your code here
hand.close()
```

### Controlling Fingers

```python
# Open and close all fingers
hand.open_all_fingers()
hand.close_all_fingers()

# Control individual fingers
hand.index_finger.open()
hand.thumb_bend.close()
hand.middle_finger.move(500)  # Move to middle position (0-1000)

# Set speed for all fingers (0-1000)
hand.set_all_finger_speeds(800)

# Set force threshold for all fingers (0-1000)
hand.set_all_finger_forces(500)
```

### Gestures

```python
# Pinch gesture (thumb and index finger)
hand.pinch(force=500)

# Pointing gesture (index finger extended)
hand.point()

# Thumbs up gesture
hand.thumbs_up()

# Grip (close all fingers with specific force)
hand.grip(force=700)
```

### Reading Status

```python
# Get current angles of all fingers
angles = hand.get_finger_angles()

# Get current force values
forces = hand.get_finger_forces()

# Get status of a specific finger
status = hand.index_finger.status
if status == FingerStatus.REACHED_FORCE:
    print("Index finger has reached the force limit")

# Check for errors
if hand.middle_finger.error & ErrorCode.OVER_TEMPERATURE:
    print("Middle finger temperature is too high!")
```

## Using Multiple Hands

If you have multiple Inspire Hands, you can control them simultaneously using this library.

### Option 1: Different Serial Ports

If your hands are connected to different USB ports, they'll be assigned different serial port names (e.g., `/dev/ttyUSB0` and `/dev/ttyUSB1`). You can create separate instances for each hand:

```python
from inspire_hand import InspireHand

# Create instances for each hand
left_hand = InspireHand("/dev/ttyUSB0")  # First hand
right_hand = InspireHand("/dev/ttyUSB1")  # Second hand

# Control them independently
left_hand.open_all_fingers()
right_hand.close_all_fingers()

# Or coordinate them for bimanual tasks
left_hand.pinch()
right_hand.grip()
```

To see which ports your hands are connected to, you can use:
```bash
ls -l /dev/ttyUSB*
```

### Option 2: Same Port with Different Slave IDs

If both hands are on the same serial bus (e.g., RS-485 daisy chain), use different slave IDs:

```python
from inspire_hand import InspireHand

# Create instances with different slave IDs
left_hand = InspireHand("/dev/ttyUSB0", slave_id=1)  # First hand with ID 1
right_hand = InspireHand("/dev/ttyUSB0", slave_id=2)  # Second hand with ID 2

# Control them individually
left_hand.thumbs_up()
right_hand.point()
```

### From the Command Line

Using the CLI with multiple hands:

```bash
# Control left hand
python -m inspire_hand --port /dev/ttyUSB0 gesture pinch

# Control right hand
python -m inspire_hand --port /dev/ttyUSB1 gesture grip
```

Or with different slave IDs:

```bash
python -m inspire_hand --port /dev/ttyUSB0 --slave-id 1 open all
python -m inspire_hand --port /dev/ttyUSB0 --slave-id 2 close all
```

## License

MIT License 
