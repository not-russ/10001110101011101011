# === SpdrBot Non-Blocking Version (Ready for Bluetooth) ===
# drop-in replacement for your old spdrbot.py

import time
from machine import Pin, PWM

# -----------------
#  CONSTANTS
# -----------------
NUM_SERVOS = 12
FREQ = 50

NEUTRAL_OFFSETS = [92, 90, 90, 90,
                   90, 90, 83, 94,
                   90, 93, 90, 90]

DIRECTION_MAP = [-1, 1, 1, 1,
                 1, 1, -1, 1,
                 1, 1, 1, 1]

JOINT_MAP = {
    "L1_a": 0, "L1_b": 1, "L1_c": 2,
    "L2_a": 3, "L2_b": 4, "L2_c": 5,
    "L3_a": 6, "L3_b": 7, "L3_c": 8,
    "L4_a": 9, "L4_b": 10, "L4_c": 11,
}

def degrees_to_duty(angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    return int((angle_deg / 180.0) * 8000 + 1000)

def ease_cubic(t):
    return 3*t*t - 2*t*t*t


# -----------------
#  SERVO CLASS
# -----------------
class Servo:
    def __init__(self, pin, offset, direction):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(FREQ)
        self.offset = offset
        self.direction = direction

        # movement state
        self.current_angle = 0
        self.start_angle = 0
        self.target_angle = 0
        self.duration = 0
        self.elapsed = 0
        self.moving = False

    def hw_set(self, angle):
        real = self.offset + angle * self.direction
        self.pwm.duty_u16(degrees_to_duty(real))

    def start(self, target, duration):
        self.start_angle = self.current_angle
        self.target_angle = target
        self.duration = max(0.001, duration)
        self.elapsed = 0
        self.moving = True

    def update(self, dt):
        if not self.moving: return
        self.elapsed += dt
        t = min(1, self.elapsed / self.duration)
        s = ease_cubic(t)
        ang = self.start_angle + (self.target_angle - self.start_angle)*s

        self.current_angle = ang
        self.hw_set(ang)

        if t >= 1:
            self.moving = False


# -----------------
#  ROBOT CLASS
# -----------------
class Robot:
    def __init__(self, pin_map):
        self.servos = [
            Servo(pin_map[i], NEUTRAL_OFFSETS[i], DIRECTION_MAP[i])
            for i in range(NUM_SERVOS)
        ]
        self.last = time.ticks_ms()

    def tick(self):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last)/1000
        self.last = now
        for s in self.servos:
            s.update(dt)

    # ---- Non-blocking setters ----
    def set_joint(self, name, angle, dur=0.4):
        idx = JOINT_MAP[name]
        self.servos[idx].start(angle, dur)

    def set_joints(self, names, angles, dur=0.4):
        for n,a in zip(names, angles):
            self.set_joint(n,a,dur)

    def set_leg(self, leg, angles, dur=0.4):
        names = [f"{leg}_a", f"{leg}_b", f"{leg}_c"]
        for n,a in zip(names, angles):
            if a is not None:
                self.set_joint(n,a,dur)

    # ---------------------------
    # Converted from OLD spdrbot.py
    # ---------------------------

    def init_position(self):
        self.set_joints(list(JOINT_MAP.keys()), [0]*NUM_SERVOS, 1.0)

    def standup(self):
        self.set_joints(["L1_a","L2_a","L3_a","L4_a"], [35,35,35,35], 1)
        self.set_joints(["L1_c","L2_c","L3_c","L4_c"], [-40,-40,-40,-40], 1)
        self.set_joints(
            ["L1_b","L2_b","L3_b","L4_b",
             "L1_c","L2_c","L3_c","L4_c"],
            [50,50,50,50,-10,-10,-10,-10], 1
        )

    def forward_step(self, leg):
        if leg in ("L1", "L2"):
            self.set_leg(leg, [40,30,-10], 0.25)
            self.set_leg(leg, [30,30,0 ], 0.25)
            self.set_leg(leg, [30,55,10], 0.25)
        else:
            self.set_leg(leg, [40,30,-10], 0.25)
            self.set_leg(leg, [50,30,0 ], 0.25)
            self.set_leg(leg, [50,55,0 ], 0.25)

    def forward_shift(self, side):
        if side == 1:
            self.set_joints(
                ["L1_a","L2_a","L3_a","L4_a",
                 "L1_b","L2_b","L3_b","L4_b",
                 "L1_c","L2_c","L3_c","L4_c"],
                [40,80,40,20,
                 50,50,50,50,
                 -10,-10,-10,-10],
                0.5)
        else:
            self.set_joints(
                ["L1_a","L2_a","L3_a","L4_a",
                 "L1_b","L2_b","L3_b","L4_b",
                 "L1_c","L2_c","L3_c","L4_c"],
                [80,40,20,40,
                 50,50,50,50,
                 -10,-10,-10,-10],
                1)

    def walk_cycle(self):
        self.forward_step("L1")
        self.forward_step("L3")
        self.forward_shift(1)
        self.forward_step("L2")
        self.forward_step("L4")
        self.forward_shift(2)

    def rotate(self):
        for leg in ("L1","L2","L3","L4"):
            self.set_leg(leg, [40,30,-10], 0.25)
            if leg in ("L1","L3"):
                self.set_leg(leg, [20,30,0], 0.25)
                self.set_leg(leg, [20,60,-10], 0.25)
            else:
                self.set_leg(leg, [60,30,0], 0.25)
                self.set_leg(leg, [60,60,-10],0.25)

        self.set_joints(["L1_a","L2_a","L3_a","L4_a"],
                        [60,60,60,60], 1)



# ------------------------------------------
#  MAIN LOOP — periodic tick (VERY IMPORTANT)
# ------------------------------------------
pins = [0,1,2,3,4,5,6,7,8,9,10,11]   # <<< CHANGE TO YOUR REAL SERVO PINS

robot = Robot(pins)
robot.init_position()
time.sleep(1)
robot.standup()

while True:
    robot.tick()     # ← keeps motion smooth
    # (Bluetooth commands will go here)
    time.sleep(0.02) # 50 Hz tick loop