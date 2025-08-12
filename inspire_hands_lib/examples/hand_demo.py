#!/usr/bin/env python3
"""
Demo script to demonstrate the usage of the Inspire Hand API.
"""

import sys
import time
import argparse
import os

# Add parent directory to import path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from inspire_hand import InspireHand, FingerStatus


def parse_args():
    parser = argparse.ArgumentParser(description="Inspire Hand Demo")
    parser.add_argument(
        "--port", "-p",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for the Inspire Hand (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=115200,
        help="Baudrate for the serial connection (default: 115200)"
    )
    parser.add_argument(
        "--demo", "-d",
        type=str,
        choices=["basic", "gestures", "fingers", "wave", "count"],
        default="basic",
        help="Demo to run (default: basic)"
    )
    return parser.parse_args()


def print_status(hand):
    """Print the current status of all fingers."""
    print("\nHand Status:")
    
    for finger in hand.fingers:
        print(f"\n{finger.name.title()} Finger:")
        print(f"  Angle: {finger.angle}")
        print(f"  Force: {finger.force}")
        print(f"  Status: {finger.status.name}")
        print(f"  Error: {finger.error}")
        print(f"  Temperature: {finger.temperature}")


def basic_demo(hand):
    """Run a basic demo of hand functionality."""
    print("\nRunning basic demo...")
    
    # Reset the hand
    print("Resetting hand...")
    hand.reset()
    time.sleep(1)
    
    # Open all fingers
    print("Opening all fingers...")
    hand.open_all_fingers()
    time.sleep(2)
    
    # Close all fingers
    print("Closing all fingers...")
    hand.close_all_fingers()
    time.sleep(2)
    
    # Open all fingers again
    print("Opening all fingers...")
    hand.open_all_fingers()
    time.sleep(2)
    
    # Print status
    print_status(hand)


def gesture_demo(hand):
    """Demonstrate predefined gestures."""
    print("\nRunning gesture demo...")
    
    # Reset and open
    print("Resetting hand...")
    hand.reset()
    time.sleep(1)
    hand.open_all_fingers()
    time.sleep(2)
    
    # Pinch gesture
    print("Pinch gesture...")
    hand.pinch(force=500)
    time.sleep(3)
    
    # Point gesture
    print("Point gesture...")
    hand.point()
    time.sleep(3)
    
    # Thumbs up gesture
    print("Thumbs up gesture...")
    hand.thumbs_up()
    time.sleep(3)
    
    # Grip gesture
    print("Grip gesture...")
    hand.grip(force=700)
    time.sleep(3)
    
    # Back to open hand
    print("Opening all fingers...")
    hand.open_all_fingers()
    time.sleep(2)


def finger_demo(hand):
    """Demonstrate individual finger control."""
    print("\nRunning individual finger demo...")
    
    # Reset and open
    print("Resetting hand...")
    hand.reset()
    time.sleep(1)
    hand.open_all_fingers()
    time.sleep(2)
    
    # Set all fingers to medium speed
    hand.set_all_finger_speeds(500)
    
    # Move each finger individually
    fingers = [
        ("thumb bend", hand.thumb_bend),
        ("thumb rotate", hand.thumb_rotate),
        ("index", hand.index_finger),
        ("middle", hand.middle_finger),
        ("ring", hand.ring_finger),
        ("little", hand.little_finger)
    ]
    
    for name, finger in fingers:
        # Close the finger
        print(f"Closing {name} finger...")
        finger.close()
        time.sleep(1.5)
        
        # Open the finger
        print(f"Opening {name} finger...")
        finger.open()
        time.sleep(1.5)
    
    # Move all fingers to middle position
    print("Moving all fingers to middle position...")
    for _, finger in fingers:
        finger.move(500)
    time.sleep(2)
    
    # Back to open hand
    print("Opening all fingers...")
    hand.open_all_fingers()
    time.sleep(2)


def wave_demo(hand):
    """Demonstrate a waving motion with the fingers."""
    print("\nRunning wave demo...")
    
    # Reset and open
    print("Resetting hand...")
    hand.reset()
    time.sleep(1)
    hand.open_all_fingers()
    time.sleep(2)
    
    # Set high speed
    hand.set_all_finger_speeds(1000)
    
    # Perform the wave motion
    fingers = [
        hand.little_finger,
        hand.ring_finger,
        hand.middle_finger,
        hand.index_finger,
        hand.thumb_bend
    ]
    
    print("Performing wave motion...")
    for _ in range(3):  # Do three waves
        # Wave closing
        for finger in fingers:
            finger.close()
            time.sleep(0.2)
        
        # Wave opening
        for finger in reversed(fingers):
            finger.open()
            time.sleep(0.2)
    
    # Back to open hand
    print("Opening all fingers...")
    hand.open_all_fingers()
    time.sleep(2)


def count_demo(hand):
    """Demonstrate counting from 1 to 5 using fingers."""
    print("\nRunning counting demo...")
    
    # Reset and close all fingers
    print("Resetting hand...")
    hand.reset()
    time.sleep(1)
    hand.close_all_fingers()
    time.sleep(2)
    
    # Set medium speed
    hand.set_all_finger_speeds(600)
    
    # Count from 1 to 5
    print("Counting from 1 to 5...")
    
    # 1 - Index finger
    print("1...")
    hand.index_finger.open()
    time.sleep(1.5)
    
    # 2 - Index and middle fingers
    print("2...")
    hand.middle_finger.open()
    time.sleep(1.5)
    
    # 3 - Index, middle, and ring fingers
    print("3...")
    hand.ring_finger.open()
    time.sleep(1.5)
    
    # 4 - Index, middle, ring, and little fingers
    print("4...")
    hand.little_finger.open()
    time.sleep(1.5)
    
    # 5 - All fingers
    print("5...")
    hand.thumb_bend.open()
    time.sleep(1.5)
    
    # Close all fingers
    print("Closing all fingers...")
    hand.close_all_fingers()
    time.sleep(2)
    
    # Open all fingers
    print("Opening all fingers...")
    hand.open_all_fingers()
    time.sleep(2)


def main():
    """Main demo function."""
    args = parse_args()
    
    try:
        with InspireHand(port=args.port, baudrate=args.baudrate) as hand:
            print(f"Connected to Inspire Hand on {args.port}")
            
            # Run the selected demo
            if args.demo == "basic":
                basic_demo(hand)
            elif args.demo == "gestures":
                gesture_demo(hand)
            elif args.demo == "fingers":
                finger_demo(hand)
            elif args.demo == "wave":
                wave_demo(hand)
            elif args.demo == "count":
                count_demo(hand)
            
            print("\nDemo completed successfully!")
            
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        return 1
    except Exception as e:
        print(f"\nError: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main()) 