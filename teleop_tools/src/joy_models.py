class Axis:
    state_ = 0

    def update(self, new_state):
        self.state_ = new_state

    def state(self):
        return self.state_


class Button:
    pressed_ = False
    clicked_ = False

    def update(self, new_state):
        if new_state and not self.pressed_:
            self.clicked_ = True
        self.pressed_ = new_state

    def clicked(self):
        if self.clicked_:
            self.clicked_ = False
            return True
        else:
            return False

    def pressed(self):
        return self.pressed_


class Joy:
    def __init__(self):
        self.buttons = {}
        for name in self.button_names:
            self.buttons[name] = Button()

        self.axes = {}
        for name in self.axis_names:
            self.axes[name] = Axis()

    def update(self, axes, buttons):
        for idx, state in enumerate(axes):
            self.axes[self.axis_names[idx]].update(state)

        for idx, state in enumerate(buttons):
            self.buttons[self.button_names[idx]].update(state)


class DualShock3Smart(Joy):
    axis_names = [
        'LEFT_STICK_LR',
        'LEFT_STICK_UD',
        'LEFT_TRIGGER',
        'RIGHT_STICK_LR',
        'RIGHT_STICK_UD',
        'RIGHT_TRIGGER']

    button_names = [
        'CROSS',
        'CIRCLE',
        'TRIANGLE',
        'SQUARE',
        'LEFT_SHIFT',
        'RIGHT_SHIFT',
        'LEFT_TRIGGER',
        'RIGHT_TRIGGER',
        'SELECT',
        'START',
        'PS',
        'LEFT_STICK',
        'RIGHT_STICK',
        'UP',
        'DOWN',
        'LEFT',
        'RIGHT']


class IPegaSmart(Joy):
    axis_names = [
        'LEFT_STICK_LR',
        'LEFT_STICK_UD',
        'LEFT_TRIGGER',
        'RIGHT_STICK_LR',
        'RIGHT_STICK_UD',
        'RIGHT_TRIGGER',
        'ARROWS_LR',
        'ARROWS_UD']

    button_names = [
        'CROSS',
        'CIRCLE',
        'SQUARE',
        'TRIANGLE',
        'LEFT_SHIFT',
        'RIGHT_SHIFT',
        'SELECT',
        'START',
        'PS',
        'LEFT_STICK',
        'RIGHT_STICK']


class PinkJoy(Joy):
    axis_names = [
        'LEFT_STICK_LR',
        'LEFT_STICK_UD',
        'LEFT_TRIGGER',
        'RIGHT_STICK_LR',
        'RIGHT_STICK_UD',
        'RIGHT_TRIGGER']

    button_names = [
        'CROSS',
        'CIRCLE',
        'SQUARE',
        'TRIANGLE',
        'LEFT_SHIFT',
        'RIGHT_SHIFT',
        'LEFT_TRIGGER',
        'RIGHT_TRIGGER',
        'SELECT',
        'START',
        'PS',
        'LEFT_STICK',
        'RIGHT_STICK',
        'ARROW_UP',
        'ARROW_DOWN',
        'ARROW_LEFT',
        'ARROW_RIGHT'
        ]


class Thrustmaster(Joy):
    axis_names = [
        'BIG_STICK_LR',
        'BIG_STICK_UD',
        'BIG_STICK_ROT',
	'SPEED',
        'SMALL_STICK_LR',
        'SMALL_STICK_UD',
	]

    button_names = [
	'START',
        'TOP_M',
        'TOP_L',
        'TOP_R',
        'LU1',
        'LU2',
        'LU3',
        'LD3',
        'LD2',
        'LD1',
        'RU3',
        'RU2',
        'RU1',
        'RD1',
        'RD2',
        'RD3',
        ]


class DUALSHOCK3:

    class AXIS:
        LEFT_STICK_LR  = 0
        LEFT_STICK_UD  = 1
        LEFT_TRIGGER   = 2
        RIGHT_STICK_LR = 3
        RIGHT_STICK_UD = 4
        RIGHT_TRIGGER  = 5

    class BUTTONS:
        CROSS         = 0
        CIRCLE        = 1
        TRIANGLE      = 2
        SQUARE        = 3
        LEFT_SHIFT    = 4
        RIGHT_SHIFT   = 5
        LEFT_TRIGGER  = 6
        RIGHT_TRIGGER = 7
        SELECT        = 8
        START         = 9
        PS            = 10
        LEFT_STICK    = 11
        RIGHT_STICK   = 12
        UP            = 13
        DOWN          = 14
        LEFT          = 15
        RIGHT         = 16


class IPega:

    class AXIS:
        LEFT_STICK_LR  = 0
        LEFT_STICK_UD  = 1
        LEFT_TRIGGER   = 2
        RIGHT_STICK_LR = 3
        RIGHT_STICK_UD = 4
        RIGHT_TRIGGER  = 5
        ARROWS_LR      = 6
        ARROWS_UD      = 7

    class BUTTONS:
        CROSS         = 0
        CIRCLE        = 1
        TRIANGLE      = 3
        SQUARE        = 2
        LEFT_SHIFT    = 4
        RIGHT_SHIFT   = 5
        SELECT        = 6
        START         = 7
        PS            = 8
        LEFT_STICK    = 9
        RIGHT_STICK   = 10

class IPega:

    class AXIS:
        LEFT_STICK_LR  = 0
        LEFT_STICK_UD  = 1
        LEFT_TRIGGER   = 2
        RIGHT_STICK_LR = 3
        RIGHT_STICK_UD = 4
        RIGHT_TRIGGER  = 5
        ARROWS_LR      = 6
        ARROWS_UD      = 7

    class BUTTONS:
        CROSS         = 0
        CIRCLE        = 1
        TRIANGLE      = 3
        SQUARE        = 2
        LEFT_SHIFT    = 4
        RIGHT_SHIFT   = 5
        SELECT        = 6
        START         = 7
        PS            = 8
        LEFT_STICK    = 9
        RIGHT_STICK   = 10


