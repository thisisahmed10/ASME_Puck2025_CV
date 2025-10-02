# motors.py
# All-in-one motor controller for Raspberry Pi 5 (Ubuntu 24.04)
# Dependencies: sudo apt install -y python3-lgpio
#
# Usage example (in your main.py):
#   import time, motors as m
#   m.set_speed(60.0)     # 0..100 % duty
#   m.forward(); time.sleep(1); m.stop()
#   m.right();   time.sleep(0.5); m.stop()
#   m.ServoRight(); time.sleep(1); m.ServoLeft()
#   m.OpenStepperGate(); m.CloseStepperGate()
#
# NOTE: Edit the CONFIG section below to match your wiring.

import time
import atexit
import lgpio

# ========== CONFIG (edit to match your wiring) ==========

# ---- DC motors (BTS7960, dual-PWM per motor) ----
# Recommended PWM-capable pins: 12, 13, 18, 19 (BCM)
LEFT_RPWM   = 12   # Left motor RPWM pin
LEFT_LPWM   = 13   # Left motor LPWM pin
RIGHT_RPWM  = 18   # Right motor RPWM pin
RIGHT_LPWM  = 19   # Right motor LPWM pin

# Optional enable pins (-1 to ignore and tie EN high on module)
LEFT_REN    = -1   # Left board R_EN (active HIGH)
LEFT_LEN    = -1   # Left board L_EN (active HIGH)
RIGHT_REN   = -1   # Right board R_EN (active HIGH)
RIGHT_LEN   = -1   # Right board L_EN (active HIGH)

DC_PWM_FREQ_HZ = 20_000     # 20 kHz (quiet)
SPEED_DUTY = 60.0           # % default "speed" you can change later (0..100)

# ---- Servo (MG995) ----
SERVO_PIN       = 21        # BCM pin for servo signal
SERVO_FREQ_HZ   = 50        # 50 Hz standard
SERVO_LEFT_US   = 1000      # microseconds pulse for "left" position
SERVO_CENTER_US = 1500      # center (not used by the two functions below, but useful)
SERVO_RIGHT_US  = 2000      # microseconds pulse for "right" position

# ---- Stepper driver (A4988/DRV8825/TB6600-style) ----
STEPPER_STEP_PIN     = 5    # STEP
STEPPER_DIR_PIN      = 6    # DIR
STEPPER_EN_PIN       = 7    # EN (active-low on many drivers; see flag below)
STEPPER_EN_ACTIVE_LOW = True
STEPPER_RATE_SPS     = 800  # steps per second
STEPPER_OPEN_STEPS   = 800  # how many steps to "open"
STEPPER_CLOSE_STEPS  = 800  # how many steps to "close"
STEPPER_IDLE_DISABLE = True # disable driver when idle

# ========================================================

_handle = None
_speed_duty = float(SPEED_DUTY)  # runtime-adjustable copy

def _open_chip():
    global _handle
    if _handle is None:
        _handle = lgpio.gpiochip_open(0)

def _claim_output(pin):
    if pin >= 0:
        lgpio.gpio_claim_output(_handle, pin)

def _set_en(pin, enable: bool, active_low=False):
    if pin < 0:
        return
    val = 0 if (enable and active_low) else 1 if enable else 1 if active_low else 0
    # Explanation:
    #   active_low: enabled=0, disabled=1
    #   active_high: enabled=1, disabled=0
    lgpio.gpio_write(_handle, pin, val)

def _init_dc():
    # Optional enable pins HIGH (enable)
    for en in (LEFT_REN, LEFT_LEN, RIGHT_REN, RIGHT_LEN):
        if en >= 0:
            _claim_output(en)
            lgpio.gpio_write(_handle, en, 1)   # BTS7960 EN is active-high

    # Start all PWM lines at 0% duty
    for pin in (LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM):
        lgpio.tx_pwm(_handle, pin, DC_PWM_FREQ_HZ, 0.0)

def _dc_pair(rpwm_pin, lpwm_pin, duty: float, forward: bool):
    """Apply duty% to the correct side of a BTS7960 pair."""
    duty = max(0.0, min(100.0, duty))
    if forward:
        lgpio.tx_pwm(_handle, rpwm_pin, DC_PWM_FREQ_HZ, duty)
        lgpio.tx_pwm(_handle, lpwm_pin, DC_PWM_FREQ_HZ, 0.0)
    else:
        lgpio.tx_pwm(_handle, rpwm_pin, DC_PWM_FREQ_HZ, 0.0)
        lgpio.tx_pwm(_handle, lpwm_pin, DC_PWM_FREQ_HZ, duty)

def _stop_dc():
    for pin in (LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM):
        lgpio.tx_pwm(_handle, pin, DC_PWM_FREQ_HZ, 0.0)

def _init_servo():
    lgpio.tx_pwm(_handle, SERVO_PIN, SERVO_FREQ_HZ, 0.0)

def _servo_pulse_us(pulse_us: int):
    # Duty% = (pulse_us / period_us) * 100
    period_us = int(1_000_000 / SERVO_FREQ_HZ)
    duty = (max(500, min(2500, int(pulse_us))) / period_us) * 100.0
    lgpio.tx_pwm(_handle, SERVO_PIN, SERVO_FREQ_HZ, duty)

def _init_stepper():
    _claim_output(STEPPER_STEP_PIN)
    _claim_output(STEPPER_DIR_PIN)
    _claim_output(STEPPER_EN_PIN) if STEPPER_EN_PIN >= 0 else None
    # By default, disable driver when idle:
    if STEPPER_EN_PIN >= 0:
        _set_en(STEPPER_EN_PIN, enable=not STEPPER_IDLE_DISABLE, active_low=STEPPER_EN_ACTIVE_LOW)

def _enable_stepper(on: bool):
    if STEPPER_EN_PIN >= 0:
        _set_en(STEPPER_EN_PIN, enable=on, active_low=STEPPER_EN_ACTIVE_LOW)

def _stepper_move(steps: int, rate_sps: int | None = None):
    """Blocking move: +N forward, -N backward."""
    n = abs(int(steps))
    if n == 0:
        return
    rate = int(rate_sps or STEPPER_RATE_SPS)
    rate = max(1, rate)
    half_period = 0.5 / rate

    # Direction: 1 for forward, 0 for reverse (you can swap if needed)
    direction = 1 if steps >= 0 else 0
    lgpio.gpio_write(_handle, STEPPER_DIR_PIN, direction)

    _enable_stepper(True)
    for _ in range(n):
        lgpio.gpio_write(_handle, STEPPER_STEP_PIN, 1)
        time.sleep(half_period)
        lgpio.gpio_write(_handle, STEPPER_STEP_PIN, 0)
        time.sleep(half_period)
    if STEPPER_IDLE_DISABLE:
        _enable_stepper(False)

# ---------- Public API (you call these from your main script) ----------

def init():
    """Call once at program start (optional; first call will auto-init)."""
    _open_chip()
    _init_dc()
    _init_servo()
    _init_stepper()

def set_speed(percent: float):
    """Change DC motor speed (0..100% duty)."""
    global _speed_duty
    _speed_duty = max(0.0, min(100.0, float(percent)))

def forward():
    _open_chip(); _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, forward=True)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, forward=True)

def backward():
    _open_chip(); _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, forward=False)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, forward=False)

def right():
    """Pivot right: left fwd, right back."""
    _open_chip(); _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, forward=True)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, forward=False)

def left():
    """Pivot left: left back, right fwd."""
    _open_chip(); _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, forward=False)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, forward=True)

def stop():
    _open_chip(); _init_dc()
    _stop_dc()

def ServoRight():
    """Move servo to 'right' preset (SERVO_RIGHT_US)."""
    _open_chip(); _init_servo()
    _servo_pulse_us(SERVO_RIGHT_US)

def ServoLeft():
    """Move servo to 'left' preset (SERVO_LEFT_US)."""
    _open_chip(); _init_servo()
    _servo_pulse_us(SERVO_LEFT_US)

def ServoCentre():
    """Move servo to center preset (SERVO_CENTER_US)."""
    _open_chip(); _init_servo()
    _servo_pulse_us(SERVO_CENTER_US)

def OpenStepperGate():
    """Open gate by STEPPER_OPEN_STEPS forward."""
    _open_chip(); _init_stepper()
    _stepper_move(+STEPPER_OPEN_STEPS)

def CloseStepperGate():
    """Close gate by STEPPER_CLOSE_STEPS backward."""
    _open_chip(); _init_stepper()
    _stepper_move(-STEPPER_CLOSE_STEPS)

def shutdown():
    """Safe stop & release GPIO (called automatically on exit)."""
    try:
        _stop_dc()
        _servo_pulse_us(0)   # stop pulses
        if STEPPER_EN_PIN >= 0:
            _enable_stepper(False)
    except Exception:
        pass
    global _handle
    if _handle is not None:
        try:
            lgpio.gpiochip_close(_handle)
        except Exception:
            pass
        _handle = None

atexit.register(shutdown)
