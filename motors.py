# motors.py — Raspberry Pi 5 (Ubuntu 24.04) motor control using lgpio
# Controls:
# - 2x DC motors via BTS7960 (forward/backward/right/left + speed)
# - 1x servo (MG995/SG90): ServoLeft/ServoCentre/ServoRight
# - 1x stepper (STEP/DIR/EN): OpenStepperGate/CloseStepperGate
#
# On Pi 5, GPIOs can be split across multiple gpiochips.
# Use `gpiofind GPIO<N>` to discover the correct chip index for each pin.

import time
import atexit
import lgpio

# =================== CONFIG (edit these to match YOUR wiring) ===================

# ---- DC motors (BTS7960, dual-PWM per motor) ----
# Tip: pins 12, 13, 18, 19 are PWM-capable (BCM numbers). Check chip index!
DC_CHIP_INDEX     = 0       # <<< run `gpiofind GPIO12` etc. and adjust later if needed
LEFT_RPWM         = 12      # left motor RPWM  (forward)
LEFT_LPWM         = 13      # left motor LPWM  (reverse)
RIGHT_RPWM        = 18      # right motor RPWM (forward)
RIGHT_LPWM        = 19      # right motor LPWM (reverse)
LEFT_REN          = -1      # optional EN pins; -1 = not used (tie EN high on module)
LEFT_LEN          = -1
RIGHT_REN         = -1
RIGHT_LEN         = -1
DC_PWM_FREQ_HZ    = 1000    # keep 500–1000 Hz for lgpio compatibility
SPEED_DUTY        = 60.0    # default speed (% duty), change via set_speed()

# ---- Servo (MG995/SG90 @ 50 Hz) ----
# You reported: gpiofind GPIO21 -> gpiochip4 21  → set SERVO_CHIP_INDEX=4
SERVO_CHIP_INDEX  = 4       # <<< from your gpiofind result
SERVO_PIN         = 21      # BCM 21 (physical pin 40)
SERVO_FREQ_HZ     = 100
SERVO_LEFT_US     = 1000    # tweak if needed (e.g., 1100)
SERVO_CENTER_US   = 1500
SERVO_RIGHT_US    = 2000    # tweak if needed (e.g., 1900)

# ---- Stepper driver (A4988/DRV8825/TB6600 style) ----
STEPPER_CHIP_INDEX   = 0    # <<< run gpiofind on 5/6/7 and adjust later
STEPPER_STEP_PIN     = 5
STEPPER_DIR_PIN      = 6
STEPPER_EN_PIN       = 7
STEPPER_EN_ACTIVE_LOW= True # many drivers are active-low (0=enabled)
STEPPER_RATE_SPS     = 800  # steps per second
STEPPER_OPEN_STEPS   = 800  # distance to open
STEPPER_CLOSE_STEPS  = 800  # distance to close
STEPPER_IDLE_DISABLE = True

# ================================================================================

_dc_handle = None
_servo_handle = None
_stepper_handle = None
_speed_duty = float(SPEED_DUTY)

# -------------------------- helpers: open/claim/close ---------------------------

def _open_dc():
    global _dc_handle
    if _dc_handle is None:
        _dc_handle = lgpio.gpiochip_open(DC_CHIP_INDEX)

def _open_servo():
    global _servo_handle
    if _servo_handle is None:
        _servo_handle = lgpio.gpiochip_open(SERVO_CHIP_INDEX)

def _open_stepper():
    global _stepper_handle
    if _stepper_handle is None:
        _stepper_handle = lgpio.gpiochip_open(STEPPER_CHIP_INDEX)

def _claim_output(handle, pin):
    if pin >= 0:
        lgpio.gpio_claim_output(handle, pin)

# --------------------------------- DC (BTS7960) ---------------------------------

def _init_dc():
    _open_dc()
    # optional enables (active-high on BTS7960)
    for en in (LEFT_REN, LEFT_LEN, RIGHT_REN, RIGHT_LEN):
        if en >= 0:
            _claim_output(_dc_handle, en)
            lgpio.gpio_write(_dc_handle, en, 1)
    # start all PWM at 0% to be safe
    for pin in (LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM):
        lgpio.tx_pwm(_dc_handle, pin, DC_PWM_FREQ_HZ, 0.0)

def _dc_pair(rpwm_pin, lpwm_pin, duty, forward: bool):
    duty = max(0.0, min(100.0, float(duty)))
    if forward:
        lgpio.tx_pwm(_dc_handle, rpwm_pin, DC_PWM_FREQ_HZ, duty)
        lgpio.tx_pwm(_dc_handle, lpwm_pin, DC_PWM_FREQ_HZ, 0.0)
    else:
        lgpio.tx_pwm(_dc_handle, rpwm_pin, DC_PWM_FREQ_HZ, 0.0)
        lgpio.tx_pwm(_dc_handle, lpwm_pin, DC_PWM_FREQ_HZ, duty)

def _stop_dc():
    if _dc_handle is None:
        return
    for pin in (LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM):
        lgpio.tx_pwm(_dc_handle, pin, DC_PWM_FREQ_HZ, 0.0)

def set_speed(percent: float):
    """Change DC speed (PWM duty)."""
    global _speed_duty
    _speed_duty = max(0.0, min(100.0, float(percent)))

def forward():
    _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, True)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, True)

def backward():
    _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, False)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, False)

def right():
    """Pivot right: left forward, right reverse."""
    _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, True)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, False)

def left():
    """Pivot left: left reverse, right forward."""
    _init_dc()
    _dc_pair(LEFT_RPWM, LEFT_LPWM, _speed_duty, False)
    _dc_pair(RIGHT_RPWM, RIGHT_LPWM, _speed_duty, True)

def stop():
    _init_dc()
    _stop_dc()

# ----------------------------------- Servo --------------------------------------

def _servo_pulse_us(pulse_us: int):
    _open_servo()
    period_us = int(1_000_000 / SERVO_FREQ_HZ)
    u = max(500, min(2500, int(pulse_us)))   # clamp safe range
    duty = (u / period_us) * 100.0
    lgpio.tx_pwm(_servo_handle, SERVO_PIN, SERVO_FREQ_HZ, duty)

def ServoLeft():
    _servo_pulse_us(SERVO_LEFT_US)

def ServoCentre():
    _servo_pulse_us(SERVO_CENTER_US)

def ServoRight():
    _servo_pulse_us(SERVO_RIGHT_US)

# ---------------------------------- Stepper -------------------------------------

def _init_stepper():
    _open_stepper()
    _claim_output(_stepper_handle, STEPPER_STEP_PIN)
    _claim_output(_stepper_handle, STEPPER_DIR_PIN)
    if STEPPER_EN_PIN >= 0:
        _claim_output(_stepper_handle, STEPPER_EN_PIN)
        # default idle state
        _stepper_enable(not STEPPER_IDLE_DISABLE)

def _stepper_enable(on: bool):
    if STEPPER_EN_PIN < 0:
        return
    # active-low handling
    val = 0 if (on and STEPPER_EN_ACTIVE_LOW) else 1 if on else 1 if STEPPER_EN_ACTIVE_LOW else 0
    lgpio.gpio_write(_stepper_handle, STEPPER_EN_PIN, val)

def _stepper_move(steps: int, rate_sps: int | None = None):
    _init_stepper()
    n = abs(int(steps))
    if n == 0:
        return
    rate = int(rate_sps or STEPPER_RATE_SPS)
    rate = max(1, rate)
    hp = 0.5 / rate

    # 1 = forward, 0 = reverse (swap if reversed for your mechanics)
    lgpio.gpio_write(_stepper_handle, STEPPER_DIR_PIN, 1 if steps >= 0 else 0)
    _stepper_enable(True)
    for _ in range(n):
        lgpio.gpio_write(_stepper_handle, STEPPER_STEP_PIN, 1)
        time.sleep(hp)
        lgpio.gpio_write(_stepper_handle, STEPPER_STEP_PIN, 0)
        time.sleep(hp)
    if STEPPER_IDLE_DISABLE:
        _stepper_enable(False)

def OpenStepperGate():
    _stepper_move(+STEPPER_OPEN_STEPS)

def CloseStepperGate():
    _stepper_move(-STEPPER_CLOSE_STEPS)

# ---------------------------------- Lifecycle -----------------------------------

def init(dc=False, servo=True, stepper=False):
    """Optional: initialize selected subsystems.
       Defaults: servo only (dc=False, servo=True, stepper=False) so tests don't fail on DC."""
    if dc: _init_dc()
    if servo: _open_servo()
    if stepper: _init_stepper()

def shutdown():
    """Safe stop & close all chips (also called via atexit)."""
    global _dc_handle, _servo_handle, _stepper_handle
    # Stop outputs where possible
    try:
        _stop_dc()
    except Exception:
        pass
    try:
        if _servo_handle is not None:
            lgpio.tx_pwm(_servo_handle, SERVO_PIN, SERVO_FREQ_HZ, 0)
    except Exception:
        pass
    try:
        if _stepper_handle is not None and STEPPER_EN_PIN >= 0:
            _stepper_enable(False)
    except Exception:
        pass
    # Close handles
    if _servo_handle is not None:
        try: lgpio.gpiochip_close(_servo_handle)
        except Exception: pass
        _servo_handle = None
    if _dc_handle is not None:
        try: lgpio.gpiochip_close(_dc_handle)
        except Exception: pass
        _dc_handle = None
    if _stepper_handle is not None:
        try: lgpio.gpiochip_close(_stepper_handle)
        except Exception: pass
        _stepper_handle = None

atexit.register(shutdown)
