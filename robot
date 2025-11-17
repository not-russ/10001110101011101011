import time
import math
import _thread # Required for running the BLE listener in the background

# Mocked imports for environments without physical hardware (uncomment for MicroPython Pico W)
# from machine import Pin, PWM 
# import ubluetooth 
# from micropython import const 

# Define constants for BLE if using ubluetooth (requires MicroPython build with BLE)
# _IRQ_CENTRAL_CONNECT = const(1)
# _IRQ_CENTRAL_DISCONNECT = const(2)
# _IRQ_GATTS_WRITE = const(3)
# _FLAG_WRITE = const(0x0008)

# UART Service UUID (Standard for serial communication)
# UART_UUID = ubluetooth.UUID(0x0001) # Placeholder UUID
# UART_TX_UUID = ubluetooth.UUID(0x0002) # Placeholder UUID
# UART_RX_UUID = ubluetooth.UUID(0x0003) # Placeholder UUID

# The service structure: Service, Characteristics (TX/RX)
# UART_SERVICE = (
#     UART_UUID,
#     (
#         (UART_TX_UUID, ubluetooth.FLAG_NOTIFY),  # Transmit (Robot -> App)
#         (UART_RX_UUID, ubluetooth.FLAG_WRITE | _FLAG_WRITE),  # Receive (App -> Robot)
#     ),
# )

# =========================================================
# === 1. CONSTANTS, GEOMETRY & TRIGONOMETRY               ===
# =========================================================

# --- Hardware Constants ---
NUM_SERVOS = 12 
FREQ = 50       

# --- Robot Geometry (Inches) ---
L_COXA = 2.0    
L_FEMUR = 3.1   
L_TIBIA = 5.5   

# --- Servo Mounting Offsets (Degrees) ---
COXA_MOUNT_OFFSET = 0.0     
FEMUR_MOUNT_OFFSET = 28.0   
TIBIA_MOUNT_OFFSET = 109.0  
TIBIA_MATH_OFFSET = TIBIA_MOUNT_OFFSET - 180.0 

# --- Calculated Foot Coordinates (Initial Position) ---
INIT_X = -1.402
INIT_Y = 0.0
INIT_Z = -5.318

# --- Joint Map (Name to Index) ---
JOINT_MAP = {
    "L1_a": 0, "L1_b": 1, "L1_c": 2, 
    "L2_a": 3, "L2_b": 4, "L2_c": 5, 
    "L3_a": 6, "L3_b": 7, "L3_c": 8, 
    "L4_a": 9, "L4_b": 10, "L4_c": 11, 
}

# --- Servo Calibration ---
NEUTRAL_OFFSETS = [92, 90, 90, 90, 90, 90, 83, 94, 90, 93, 90, 90]
DIRECTION_MAP = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1] 

# Global variable to hold commands received from BLE thread
received_command = None 

# --- Helper Functions ---

def degrees_to_duty(angle_deg):
    """Converts a standard 0-180 degree angle to a 16-bit PWM duty cycle (0-65535)."""
    angle_deg = max(0, min(180, angle_deg))
    # This formula is specific to MicroPython PWM resolution (0-65535)
    return int((angle_deg / 180.0) * 8000 + 1000) # Placeholder values

def ease_cubic(t):
    """Cubic easing function for smooth motion."""
    return 3*t*t - 2*t*t*t

def get_leg_ik(x, y, z):
    """
    INVERSE KINEMATICS (IK) CORE FUNCTION
    Calculates the three servo angles required for a foot coordinate (x, y, z).
    (Heavy comments removed here to save space, but were included in the previous file.)
    """
    try:
        # 1. Top-Down View (Coxa Servo: angle_a)
        phys_angle_a = math.degrees(math.atan2(y, x))
        L_horiz_total = math.sqrt(x**2 + y**2)
        L_horiz_eff = L_horiz_total - L_COXA
        
        # 2. Side View (2-Link Arm Problem)
        L_eff = math.sqrt(L_horiz_eff**2 + z**2)
        max_reach = L_FEMUR + L_TIBIA
        min_reach = abs(L_FEMUR - L_TIBIA)
        if L_eff > max_reach or L_eff < min_reach:
            # print(f"IK Error: Target is unreachable. Distance: {L_eff:.2f}, Max: {max_reach:.2f}")
            return None

        # Solve for D1 (angle between L_FEMUR and L_eff) using Law of Cosines
        cos_D1 = (L_eff**2 + L_FEMUR**2 - L_TIBIA**2) / (2 * L_eff * L_FEMUR)
        cos_D1 = max(-1.0, min(1.0, cos_D1)) 
        D1 = math.degrees(math.acos(cos_D1))
        D2 = math.degrees(math.atan2(z, L_horiz_eff))
        phys_angle_b = D1 + D2
        
        # Solve for D3 (internal angle between Femur and Tibia) using Law of Cosines
        cos_D3 = (L_FEMUR**2 + L_TIBIA**2 - L_eff**2) / (2 * L_FEMUR * L_TIBIA)
        cos_D3 = max(-1.0, min(1.0, cos_D3)) 
        D3 = math.degrees(math.acos(cos_D3))
        phys_angle_c = D3 - 180.0
        
        # 3. Apply Servo Mounting Offsets 
        servo_angle_a = phys_angle_a - COXA_MOUNT_OFFSET
        servo_angle_b = phys_angle_b - FEMUR_MOUNT_OFFSET
        servo_angle_c = phys_angle_c - TIBIA_MATH_OFFSET
        
        return (servo_angle_a, servo_angle_b, servo_angle_c)

    except ValueError:
        return None

# =========================================================
# === 2. SERVO CLASS (Individual Servo Controller)        ===
# =========================================================
# (Servo class remains identical to the previous file. It manages the movement state and PWM control.)

class Servo:
    def __init__(self, pin, offset, direction):
        # self.pwm = PWM(Pin(pin)) # Initialize PWM hardware
        # self.pwm.freq(FREQ)
        self.offset = offset       
        self.direction = direction 
        
        self.current_angle = 0.0   
        self.start_angle = 0.0     
        self.target_angle = 0.0    
        self.duration = 0.0        
        self.elapsed = 0.0         
        self.moving = False        

        self.hw_set(0) 

    def hw_set(self, angle):
        """Converts the calculated angle into PWM duty cycle and commands hardware."""
        real_angle = self.offset + angle * self.direction
        # self.pwm.duty_u16(degrees_to_duty(real_angle)) 
        pass 

    def start_move(self, target, duration):
        """Initializes a new non-blocking move to a target angle."""
        self.start_angle = self.current_angle
        self.target_angle = target
        self.duration = max(0.001, duration) 
        self.elapsed = 0.0
        self.moving = True

    def update(self, dt):
        """Called every tick. Calculates the smooth intermediate position."""
        if not self.moving: return
        
        self.elapsed += dt
        t = min(1.0, self.elapsed / self.duration) 
        s = ease_cubic(t) 
        
        ang = self.start_angle + (self.target_angle - self.start_angle) * s

        self.current_angle = ang
        self.hw_set(ang) 

        if t >= 1.0:
            self.moving = False
            self.current_angle = self.target_angle 


# =========================================================
# === 3. ROBOT CLASS (Gait Engine and Control)            ===
# =========================================================
class Robot:
    def __init__(self, pin_map):
        self.servos = [
            Servo(pin_map[i], NEUTRAL_OFFSETS[i], DIRECTION_MAP[i])
            for i in range(NUM_SERVOS)
        ]
        self.last_tick_ms = time.ticks_ms()

        self.move_queue = []
        self.queue_timer = 0.0
        self.queue_running = False
        self.current_state = "STOPPED" 


    def tick(self):
        """
        HEARTBEAT FUNCTION. Called constantly to update time and servo positions.
        """
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_tick_ms) / 1000.0 
        self.last_tick_ms = now
        
        # 1. Update all 12 servo positions
        for s in self.servos:
            s.update(dt)

        # 2. Process the move queue (Gait Engine)
        if self.queue_running:
            self.queue_timer -= dt
            
            if self.queue_timer <= 0:
                if self.move_queue:
                    (names, angles, duration) = self.move_queue.pop(0)
                    self._start_servo_move(names, angles, duration)
                    self.queue_timer = duration 
                else:
                    self.queue_running = False
                    self.current_state = "READY"


    def is_busy(self):
        """Returns True if the robot is currently executing a queue or a move."""
        return self.queue_running

    def queue_clear(self):
        """Immediately stops all queued motions and pending moves."""
        self.move_queue = []
        self.queue_running = False
        self.current_state = "STOPPED"

    def _start_servo_move(self, names, angles, duration):
        """Internal helper to command the physical servos."""
        for n, a in zip(names, angles):
            idx = JOINT_MAP[n]
            self.servos[idx].start_move(a, duration)

    def queue_move_angles(self, names, angles, duration=0.4):
        """Adds a multi-joint move command (raw angles) to the execution queue."""
        self.move_queue.append((names, angles, duration))
        if not self.queue_running:
            self.queue_running = True
            self.queue_timer = 0 

    def set_foot_position(self, leg, x, y, z, duration=0.4):
        """
        Public Method: Calculates IK for a target (x,y,z) and queues the resulting servo move.
        """
        angles = get_leg_ik(x, y, z)
        
        if angles is None:
            return

        angle_a, angle_b, angle_c = angles
        names = [f"{leg}_a", f"{leg}_b", f"{leg}_c"]
        final_angles = [angle_a, angle_b, angle_c]
        
        self.queue_move_angles(names, final_angles, duration)


    # =========================================================
    # === 4. HIGH-LEVEL GAIT SEQUENCES                        ===
    # =========================================================

    def init_position(self, duration=1.0):
        """Moves all four legs to the calculated curled-up starting coordinate (0, 90, 90)."""
        self.queue_clear()
        print(f"Queueing move to INIT position: ({INIT_X:.2f}, {INIT_Y:.2f}, {INIT_Z:.2f})")
        
        for leg_name in ("L1", "L2", "L3", "L4"):
             self.set_foot_position(leg_name, INIT_X, INIT_Y, INIT_Z, duration)
        self.current_state = "INITING"


    def standup(self, target_z=-4.0, duration=1.5):
        """Stands the robot up by moving all feet to a higher Z-coordinate in sync."""
        self.queue_clear()
        print(f"Queueing STANDUP to Z={target_z}...")
        
        for leg_name in ("L1", "L2", "L3", "L4"):
             self.set_foot_position(leg_name, INIT_X, INIT_Y, target_z, duration)
        self.current_state = "STANDING_UP"


    def forward_step(self, leg, step_length=3.0, step_height=1.0, stand_z=-4.0, duration=0.5):
        """Queues the 3 steps needed for a single foot to lift, move forward, and land."""
        current_x = INIT_X 
        lift_z = stand_z + step_height 
        target_x = current_x + step_length 

        # Step 1: LIFT the foot (Move Z up)
        self.set_foot_position(leg, current_x, INIT_Y, lift_z, duration / 3)
        
        # Step 2: SWING the foot (Move X forward while high)
        self.set_foot_position(leg, target_x, INIT_Y, lift_z, duration / 3)
        
        # Step 3: PLACE the foot (Move Z down)
        self.set_foot_position(leg, target_x, INIT_Y, stand_z, duration / 3)


    def walk_cycle(self, duration=0.8):
        """
        Executes a 4-leg gait, ensuring 1 leg moves completely before the next begins.
        The queue ensures this sequential execution.
        """
        # Note: We rely on the control loop to call queue_clear() if busy.
        print("Queueing WALK CYCLE (1 Leg at a Time)...")
        
        # Gait sequence: L1, L3, L2, L4 (Simple Ripple/Trot Test)
        self.forward_step("L1", duration=duration) 
        self.forward_step("L3", duration=duration) 
        self.forward_step("L2", duration=duration) 
        self.forward_step("L4", duration=duration) 

        self.current_state = "WALKING"

    
    # =========================================================
    # === 5. CONTROL FUNCTION (User Interface)                ===
    # =========================================================

    def run_command(self, command):
        """Processes a high-level command string and executes the corresponding sequence."""
        cmd = command.lower().strip()
        print(f"Received Command: {cmd}")

        # Stop is the only command allowed while busy
        if self.is_busy() and cmd not in ["stop", "clear"]:
            print("Robot is busy executing a previous command.")
            return

        if cmd == "stop" or cmd == "clear":
            self.queue_clear()
            self.init_position(duration=0.5) # Quick drop to init position
            self.current_state = "STOPPED"
        elif cmd == "stand":
            self.standup(target_z=-4.0, duration=1.5)
        elif cmd == "walk":
            # Clear existing queue and start new walk
            self.queue_clear()
            self.walk_cycle(duration=0.8)
        else:
            print(f"Unknown command: {command}")


# =========================================================
# === 6. BLUETOOTH LOW ENERGY (BLE) FRAMEWORK             ===
# =========================================================

# Lock to protect the global 'received_command' variable from race conditions
command_lock = _thread.allocate_lock()


def ble_task():
    """
    Runs in a separate thread. Initializes the BLE stack and listens for commands.
    NOTE: You MUST adjust the BLE service UUIDs and setup for your specific environment.
    """
    global received_command

    # --- 1. BLE Initialization (Uncomment for Pico W) ---
    # ble = ubluetooth.BLE()
    # ble.active(True)
    # ((tx_handle, rx_handle),) = ble.gatts_register_services((UART_SERVICE,))
    # ble.gap_advertise(100_000, adv_data=b'\x02\x01\x06' + b'\x05\x09' + b'SPDR')
    # print("BLE Advertising started as 'SPDR'")

    # def ble_irq(event, data):
    #     if event == _IRQ_CENTRAL_CONNECT:
    #         print("BLE Connected.")
    #     elif event == _IRQ_CENTRAL_DISCONNECT:
    #         print("BLE Disconnected. Re-advertising.")
    #         ble.gap_advertise(100_000, adv_data=b'\x02\x01\x06' + b'\x05\x09' + b'SPDR')
    #     elif event == _IRQ_GATTS_WRITE:
    #         # A central device wrote to the RX characteristic
    #         handle, value_handle, char_data = data
    #         if value_handle == rx_handle:
    #             cmd_bytes = ble.gatts_read(rx_handle)
    #             cmd_str = cmd_bytes.decode('utf-8').strip()
    #             
    #             with command_lock:
    #                 received_command = cmd_str
    
    # ble.irq(ble_irq)
    # --- End BLE Initialization ---

    # --- 2. Mock Listener Loop (for testing without Pico W) ---
    print("BLE Thread Mock Started. Use simple 'stand'/'walk'/'stop' commands.")
    
    # Example loop that waits for user input, simulating a received command
    while True:
        try:
            # In a real scenario, this is where the ble.irq callback would set received_command
            # time.sleep(0.1) 

            # For local testing, use input()
            test_cmd = input("Enter command ('stand', 'walk', 'stop'): ")
            with command_lock:
                 received_command = test_cmd

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"BLE Mock Error: {e}")
            time.sleep(1)
    print("BLE Thread Mock Exited.")


# =========================================================
# === 7. MAIN APPLICATION ENTRY POINT                     ===
# =========================================================

# <<< CRITICAL: CHANGE THESE PINS TO YOUR ACTUAL WIRING >>>
pins = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] 

# Initialize robot
robot = Robot(pins)

# Start the BLE listener in a separate thread
_thread.start_new_thread(ble_task, ())

print("SpdrBot Main Loop Running. Waiting for commands from BLE thread...")


while True:
    
    # -----------------------------------------------------------
    # 1. COMMAND PROCESSING (Checks for input from the BLE thread)
    # -----------------------------------------------------------
    current_command = None
    with command_lock:
        if received_command:
            current_command = received_command
            received_command = None # Clear the command after reading it

    if current_command:
        # Pass the received command directly to the robot's control function
        robot.run_command(current_command)
        
    # -----------------------------------------------------------
    # 2. HEARTBEAT (The Core)
    # -----------------------------------------------------------
    robot.tick()
    
    # This sleep rate determines the tick frequency (50Hz = 20ms)
    time.sleep_ms(20) 