"""
servo_pwm.py -- Physical PWM ranges for all RAWES servo outputs.

Single source of truth for servo PWM limits.  Import these constants
wherever PWM values are constructed, normalised, or interpreted.
"""

# Swashplate servos S1/S2/S3 (SERVO1-3, MAIN OUT 1-3)
# ArduPilot params: H_COL_MIN=1000 / H_COL_MAX=2000 (full range, 1:1 Lua mapping)
SWASH_PWM_MIN     = 1000   # µs
SWASH_PWM_NEUTRAL = 1500   # µs
SWASH_PWM_MAX     = 2000   # µs
SWASH_PWM_RANGE   =  500   # µs — half-range from neutral to rail

# GB4008 anti-rotation motor (SERVO4, MAIN OUT 4, H_TAIL_TYPE=4 DDFP)
# ArduPilot params: SERVO4_MIN=800 / SERVO4_MAX=2000 / SERVO4_TRIM=800
MOTOR_PWM_MIN     =  800   # µs — motor off
MOTOR_PWM_MAX     = 2000   # µs — full throttle

# Motor interlock channel (CH8 / SERVO8)
INTERLOCK_PWM_LOW  = 1000  # µs — interlock OFF (motor disabled)
INTERLOCK_PWM_HIGH = 2000  # µs — interlock ON  (motor enabled)
