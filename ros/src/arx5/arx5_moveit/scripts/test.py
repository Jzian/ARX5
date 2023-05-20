# #!/usr/bin/env python3
# import rospy
# from arx5_control.control import RoboticArmAgent

# if __name__ == "__main__":
#     # 初始化机器人体
#     NODE_NAME = 'test'
#     rospy.init_node(NODE_NAME)
#     rospy.loginfo("Initializing {} Node.".format(NODE_NAME))    
#     arm = RoboticArmAgent(True,init_pose=1,excute_mode=0)
#     rospy.loginfo("{} Node Ready.".format(NODE_NAME))
#     rospy.spin()


# import sys
# import tty
# import termios
# import select
# import threading

# def set_raw_mode():
#     """Put the terminal into raw mode."""
#     stdin_fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(stdin_fd)
#     tty.setraw(stdin_fd)
#     return old_settings

# def restore_settings(old_settings):
#     """Restore the terminal's original settings."""
#     stdin_fd = sys.stdin.fileno()
#     termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)

# def get_input():
#     """Get a single character of input from the user, if available."""
#     rlist, _, _ = select.select([sys.stdin], [], [], 0)
#     if rlist:
#         return sys.stdin.read(1)
#     else:
#         return None

# def input_wrap(func):
#     """Wrap a function with input capturing and 'ctrl + c' handling."""
#     def wrapper():
#         old_settings = set_raw_mode()
#         try:
#             while True:
#                 key = get_input()
#                 if key is not None:
#                     if key == '\x03':  # 'ctrl + c'
#                         print("\nExiting...")
#                         sys.exit(0)
#                     else:
#                         func(key)
#         finally:
#             restore_settings(old_settings)
#     return wrapper

# @input_wrap
# def handle_key(key):
#     """Handle a single character of keyboard input."""
#     print('okkkk',key, end='', flush=True)

# if __name__ == '__main__':
#     print("Press 'ctrl + c' to exit.")
#     handle_key()  # Call the wrapped function instead of print_key()

import sys
import tty
import termios

def set_raw_mode():
    """Put the terminal into raw mode."""
    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    tty.setraw(stdin_fd)
    return old_settings

def restore_settings(old_settings):
    """Restore the terminal's original settings."""
    stdin_fd = sys.stdin.fileno()
    termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)

def read_key():
    """Read a single key from the terminal."""
    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    try:
        tty.setraw(stdin_fd)
        key = sys.stdin.read(1)
        if key == '\x03':  # 'ctrl + c'
            # Read the second character for a 'ctrl + c' combination
            tty.setraw(stdin_fd)
            second_key = sys.stdin.read(1)
            if second_key == '\x03':
                key = '\x03\x03'  # 'ctrl + c' combination
            else:
                key += second_key
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
    return key

if __name__ == '__main__':
    print("Press 'ctrl + c' to exit.")
    while True:
        key = read_key()
        if key == '\x03\x03':
            print("\nExiting...")
            sys.exit(0)
        elif key:
            print("Key pressed: ", key)