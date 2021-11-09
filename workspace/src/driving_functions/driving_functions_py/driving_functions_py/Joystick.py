#
# MIT License
#
# Copyright (c) 2018-2022 Wisconsin Autonomous
#
# See https://wa.wisc.edu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import array
import struct
from fcntl import ioctl
from threading import Thread

# Adopted from https://gist.github.com/rdb/8864666


class Joystick:

    # These constants were borrowed from linux/input.h
    axis_names = {
        0x00: "x",
        0x01: "y",
        0x02: "z",
        0x03: "rx",
        0x04: "ry",
        0x05: "rz",
        0x06: "throttle",
        0x07: "rudder",
        0x08: "wheel",
        0x09: "gas",
        0x0A: "brake",
        0x10: "hat0x",
        0x11: "hat0y",
        0x12: "hat1x",
        0x13: "hat1y",
        0x14: "hat2x",
        0x15: "hat2y",
        0x16: "hat3x",
        0x17: "hat3y",
        0x18: "pressure",
        0x19: "distance",
        0x1A: "tilt_x",
        0x1B: "tilt_y",
        0x1C: "tool_width",
        0x20: "volume",
        0x28: "misc",
    }

    button_names = {
        0x120: "trigger",
        0x121: "thumb",
        0x122: "thumb2",
        0x123: "top",
        0x124: "top2",
        0x125: "pinkie",
        0x126: "base",
        0x127: "base2",
        0x128: "base3",
        0x129: "base4",
        0x12A: "base5",
        0x12B: "base6",
        0x12F: "dead",
        0x130: "a",
        0x131: "b",
        0x132: "c",
        0x133: "x",
        0x134: "y",
        0x135: "z",
        0x136: "tl",
        0x137: "tr",
        0x138: "tl2",
        0x139: "tr2",
        0x13A: "select",
        0x13B: "start",
        0x13C: "mode",
        0x13D: "thumbl",
        0x13E: "thumbr",
        0x220: "dpad_up",
        0x221: "dpad_down",
        0x222: "dpad_left",
        0x223: "dpad_right",
        # XBox 360 controller uses these codes.
        0x2C0: "dpad_left",
        0x2C1: "dpad_right",
        0x2C2: "dpad_up",
        0x2C3: "dpad_down",
    }

    def __init__(self):

        # We'll store the states here.
        self.axis_states = {}
        self.button_states = {}

        self.axis_map = []
        self.button_map = []

        # Open the joystick device.
        self.fn = "/dev/input/js0"
        self.jsdev = open(self.fn, "rb")

        # Get the device name.
        buf = array.array("B", [0] * 64)
        ioctl(
            self.jsdev, 0x80006A13 + (0x10000 * len(buf)), buf
        )  # JSIOCGNAME(len)
        # js_name = buf.tobytes().rstrip(b"\x00").decode("utf-8")
        # print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array("B", [0])
        ioctl(self.jsdev, 0x80016A11, buf)  # JSIOCGAXES
        self.num_axes = buf[0]

        buf = array.array("B", [0])
        ioctl(self.jsdev, 0x80016A12, buf)  # JSIOCGBUTTONS
        self.num_buttons = buf[0]

        # Get the axis map.
        buf = array.array("B", [0] * 0x40)
        ioctl(self.jsdev, 0x80406A32, buf)  # JSIOCGAXMAP

        for axis in buf[: self.num_axes]:
            axis_name = self.axis_names.get(axis, "unknown(0x%02x)" % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array("H", [0] * 200)
        ioctl(self.jsdev, 0x80406A34, buf)  # JSIOCGBTNMAP

        for btn in buf[: self.num_buttons]:
            btn_name = self.button_names.get(btn, "unknown(0x%03x)" % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        self.updateThread = Thread(target=self.updateValues)
        self.updateThread.start()

    def getJoystickValues(self):
        return {
            "throttle": self.axis_states["rz"],
            "braking": self.axis_states["z"],
            "steering": self.axis_states["rx"],
        }

    # Main event loop
    def updateValues(self):
        while True:
            evbuf = self.jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack("IhBB", evbuf)

                if type & 0x02:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        # print("%s: %.3f" % (axis, fvalue))
