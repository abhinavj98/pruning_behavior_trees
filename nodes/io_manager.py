import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from functools import partial

class Button:
    def __init__(
        self, off_state=False, on_callback=None, off_callback=None, switch_on_callback=None, switch_off_callback=None
    ):
        """
        Initialize the Button class.

        Args:
        - off_state (bool): The default "off" state of the button.
        - on_callback (callable): Function to call when the button is in the "on" state and remains unchanged.
        - off_callback (callable): Function to call when the button is in the "off" state and remains unchanged.
        - switch_on_callback (callable): Function to call when the button switches to the "on" state.
        - switch_off_callback (callable): Function to call when the button switches to the "off" state.
        """
        self.off_state = off_state  # Default "off" state of the button.
        self.current_state = off_state  # Current state of the button.

        # Callbacks for different button states.
        self.on_callback = on_callback
        self.off_callback = off_callback
        self.switch_on_callback = switch_on_callback
        self.switch_off_callback = switch_off_callback

    def process(self, state):
        """
        Process a new state for the button.

        Args:
        - state (bool): The current state of the button (on or off).
        """
        is_on = state != self.off_state  # Determine if the button is in the "on" state.
        
        # Check if the state has changed.
        if self.current_state != state:
            # If the button is switched "on".
            if is_on:
                self.run_callback(self.switch_on_callback)
            # If the button is switched "off".
            else:
                self.run_callback(self.switch_off_callback)
            
            # Update the current state to the new state.
            self.current_state = state
        else:
            # If the state hasn't changed, run the appropriate callback.
            if is_on:
                self.run_callback(self.on_callback)
            else:
                self.run_callback(self.off_callback)

    def run_callback(self, cb):
        """
        Execute the provided callback if it exists.

        Args:
        - cb (callable): The callback function to execute.
        """
        if cb is not None:
            cb()

class Axis:
    def __init__(self, low_deadzone, high_deadzone, low_callback=None, high_callback=None):
        self.low_dz = low_deadzone
        self.high_dz = high_deadzone
        self.low_callback = low_callback
        self.high_callback = high_callback

        self.current_state = 0
        return

    def process(self, state):
        mode = 0
        if state <= self.low_dz:
            mode = -1
        if state >= self.high_dz:
            mode = 1

        if mode != self.current_state:
            self.current_state = mode
            if mode == 1 and self.high_callback is not None:
                self.high_callback()
            elif mode == -1 and self.low_callback is not None:
                self.low_callback()
        return


class IOManager(Node):
    def __init__(self):
        super().__init__("io_manager")

        self.callback_group = ReentrantCallbackGroup()
        self.action_pub = self.create_publisher(Int16, "/joy_action", 1, callback_group=self.callback_group)
        self.button_sub = self.create_subscription(Joy, "/joy", self.handle_joy, 1, callback_group=self.callback_group)
        
        """
        xbox_controller = {
            "buttons": {
                0: "A",
                1: "B",
                2: "X",
                3: "Y",
                4: "LB",
                5: "RB",
                6: "view_button",
                7: "menu_button",
                8: "xbox_button",
                9: "left_joystick",
                10: "right_joystick",
                11: "share_button"
            },
            "axes": {
                0: "left_joy_x",
                1: "left_joy_y",
                2: "LT",
                3: "right_joy_x",
                4: "right_joy_y",
                5: "RT",
                6: "Dpad_x", #18.
                7: "Dpad_y" #19
            }
        }
        """

        #Assign on and off callbacks to the buttons
        self.buttons = {i:Button(
                off_state=False, 
                switch_on_callback=partial(self.send_joy_action, i+1), 
                switch_off_callback=partial(self.send_joy_action, -(i+1))
            ) for i in range(14)}
        
        #Assign low and high callbacks to the axes
        self.axes = {
            6:Axis(-1.0, 1.0, high_callback=partial(self.send_joy_action, 18), low_callback=partial(self.send_joy_action, -18)),
            7:Axis(-1.0, 1.0, high_callback=partial(self.send_joy_action, 19), low_callback=partial(self.send_joy_action, -19))
        }
        
        return

    def handle_joy(self, msg: Joy):
        for i, state in enumerate(msg.buttons):
            if i in self.buttons:
                self.buttons[i].process(bool(state))

        for i, state in enumerate(msg.axes):
            if i in self.axes:
                self.axes[i].process(state)
        return

    def send_joy_action(self, val):
        self.action_pub.publish(Int16(data=val))
        return

def main(args=None):
    rclpy.init(args=args)
    io_manager = IOManager()
    rclpy.spin(io_manager)
    rclpy.shutdown()
if __name__ == "__main__":
    main()