#!/usr/bin/env python

from sensor_msgs.msg import Joy

class JoystickInterface(object):
    """
    Joystick abstraction layer that allows to specify multiple buttons/axes per single functionality.

    In the constructor you specify the "configuration" of the joystick you want to use, which comprises of
    functionalities and button/axis assignments for each functionality. The functionalities are the keys of 
    the config dictionary. Every key has a list assigned to it, in which are listed all buttons/axes that
    control that functionality (in the order of precedence).

    When correctly set up, you are expected to periodically call get_value() to ask for values for the
    functionalities. Using this library, you can "remap" any joystick hardware values to any desired
    "functionality value range", and in your code you then just cleanly ask for the values of your 
    functionalities. You can also "mix" two (or more) buttons/axes into one functionality.

    See config/readme.yaml for detailed description of the available options. Writing and loading a YAML file
    is also considered to be the easiest way to get the config dictionary you have to pass to the constructor 
    (e.g. by loading the YAML file into a dictionary).
    """

    def __init__(self, config):
        """
        Set up the interface using the given joystick configuration.
        @param config: The configuration of the joystick. See config/readme.yaml for all available options.
        @type config: dict
        """
        self.config = config

        # This maps the "types" from config to functions to process the values. We only have types "button" and "axis"
        # in sensor_msgs.msg.Joy, but be prepared for extension :) Feel free to add to this list from a subclass.
        self.get_value_functions = {'button': self.get_button_value, 'axis': self.get_axis_value}

        # If the config contains key dead_man, tell the class we want to use the dead man's button.
        self.dead_man = ('dead_man' in config) and (len(config['dead_man']) > 0)

        # This is a workaround for axis keys that publish other than neutral value before the axis is moved for
        # the first time. This is e.g. the case of Logitech F710 back side axes in XInput mode.
        self.startup_block_passed = {}

    def get_value(self, joy, key):
        """
        Return the value for a specified config key for the given joystick state.
        @param joy: The state of the joystick to compute the value from.
        @type joy: sensor_msgs.msg.Joy
        @param key: The config key to get value for. It has to be a key from the config passed to constructor.
        @type key: string
        @rtype: object
        """
        if not key in self.config:
            raise ValueError("Key " + key + " not found in joy interface config.")

        # If there is a dead man's button specified, check, if it is pressed. If not, just return the neutral value.

        if self.dead_man and key != 'dead_man':
            dead_man_neutral = self.get_neutral_value_for_key('dead_man')
            if self.get_value(joy, 'dead_man') == dead_man_neutral:
                return self.get_neutral_value_for_key(key)

        # Iterate over all the buttons/axes related to the given config key. The first one which gives a non-neutral
        # value will be considered as pressed. All other buttons/axes are assumed to give neutral values.
        for control in self.config[key]:
            if not 'type' in control:
                raise ValueError("Type not specified for a control definition for key " + key)

            control_type = control['type']
            if control_type in self.get_value_functions:
                # Extract the button's value.
                result = self.get_value_functions[control_type](joy, control)

                neutral_value = self.get_value_or_default(control, 'neutral_value', 0)

                # Only return if we got a non-neutral value. Otherwise give a chance to the other butons defined for
                # this config key.
                if result != neutral_value:
                    return result
            else:
                raise ValueError("Type " + control_type + " is unknown.")

        # If all defined keys return neutral values (nothing is pressed), return the neutral value.
        return self.get_neutral_value_for_key(key)

    def get_neutral_value_for_key(self, key):
        """
        Return the value that is considered neutral for the given config key.
        @param key: The config key to get neutral value for.
        @type key: string
        @rtype: object
        """
        # When no buttons are defined for the key, return zero. It could also raise an exception.
        if len(self.config[key]) == 0:
            return 0

        # Otherwise return the neutral value specified for the first button.
        control = self.config[key][0]
        return self.get_value_or_default(control, 'neutral_value', 0)

    def get_button_value(self, joy, control):
        """
        Value getter for the "button" type. Reads joy.buttons.
        @param joy: The current joystick state to get value for.
        @type joy: sensor_msgs.msg.Joy
        @param control: Specifictaion of the button parameters. See config/readme.yaml for all possible parameters.
        @type control: dict
        @rtype: object
        """
        button_state = joy.buttons[control['id']]

        if button_state == 1:
            return self.get_value_or_default(control, 'on_value', 1)
        else:
            return self.get_value_or_default(control, 'off_value', 0)

    def get_axis_value(self, joy, control):
        """
        Value getter for the "axis" type. Reads joy.axes.
        @param joy: The current joystick state to get value for.
        @type joy: sensor_msgs.msg.Joy
        @param control: Specifictaion of the axis parameters. See config/readme.yaml for all possible parameters.
        @type control: dict
        @rtype: object
        """
        # The value provided from ROS.
        hw_value = joy.axes[control['id']]

        ###
        # STARTUP VALUE WORKAROUND START
        ###

        # None means there is no need to apply the startup value workaround. See self.startup_block_passed.
        startup_block_value = self.get_value_or_default(control, 'startup_block_value', None)

        # If we have to use the startup value workaround, then continue returning the neutral value as long as the
        # value hasn't changed; once we receive any other value, we remember in self.startup_block_passed that this
        # axis is now "clean" and we can trust its values.
        if startup_block_value is not None and control['id'] not in self.startup_block_passed and hw_value == startup_block_value:
            return self.get_value_or_default(control, 'neutral_value', 0)

        # Either we don't need the workaround or the axis value is different from the startup one, which means 
        # we want to remember this axis as "clean".
        self.startup_block_passed[control['id']] = True

        ###
        # STARTUP VALUE WORKAROUND END
        ###

        # Mimimum value from the joystick.
        hw_min = float(self.get_value_or_default(control, 'hw_min', -1))
        # Maximum value from the joystick.
        hw_max = float(self.get_value_or_default(control, 'hw_max', 1))
        # Minimum output value.
        remap_min = float(self.get_value_or_default(control, 'remap_min', -1))
        # Maximum output value.
        remap_max = float(self.get_value_or_default(control, 'remap_max', 1))
        # If True, the joystick min maps to output max and vice versa.
        invert = self.get_value_or_default(control, 'invert', False)

        hw_range = hw_max - hw_min
        remap_range = remap_max - remap_min
        
        if invert:
            value = hw_max - hw_value
        else:
            value = hw_value - hw_min

        # Scale and shift the joystick value to the output range.
        return value * remap_range / hw_range + remap_min


    def get_value_or_default(self, control, key, default):
        """
        If "key" is a key in "control", return its associated value. Otherwise return "default".
        @param control: The dictionary to get values from.
        @type control: dict
        @param key: The key to search for in "control".
        @type key: string
        @param default: The value to return if "key" is not a key in "control".
        @type default: object
        @rtype: object
        """
        if key in control:
            return control[key]
        else:
            return default

    def get_description(self):
        """
        Return the description of the loaded configuration, and optionally provide ASCII-art drawing of the functions.
        @rtype: string
        """
        if 'description' in self.config and self.config['description'] != '':
            return self.config['description']
        else:
            return "The current joystick config has no description specified. Please, specify key 'description' in the config to show the description message (or drawing) here."
