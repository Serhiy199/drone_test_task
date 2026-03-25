

from dronekit import connect, VehicleMode
import time
import math
import argparse

# =====================
# ПІДКЛЮЧЕННЯ
# =====================
parser = argparse.ArgumentParser()
parser.add_argument("--connect")
args = parser.parse_args()

connection_string = args.connect if args.connect else "127.0.0.1:14551"

print(f"Connecting to {connection_string}")
vehicle = connect(connection_string, wait_ready=True)

# =====================
# ВІТЕР
# =====================
vehicle.parameters["SIM_WIND_SPD"] = 0
vehicle.parameters["SIM_WIND_DIR"] = 30
vehicle.parameters["SIM_WIND_TURB"] = 2

print("Wind configured")

# =====================
# ЦІЛЬ
# =====================
TARGET_LAT = 50.443326
TARGET_LON = 30.448078
TARGET_ALT = 100.0

# =====================
# RC НАЛАШТУВАННЯ
# =====================
RC_ROLL_NEUTRAL = 1500
RC_PITCH_NEUTRAL = 1500
RC_YAW_NEUTRAL = 1500

RC_THROTTLE_HOVER = 1398   
RC_THROTTLE_MIN = 1200
RC_THROTTLE_MAX = 1480

# 🔥 швидкість ~5 м/с
PITCH_FORWARD_FAST = 1200
PITCH_FORWARD_MED = 1280
PITCH_FORWARD_SLOW = 1380
PITCH_BACK = 1560

YAW_MIN = 1400
YAW_MAX = 1600

# =====================
# МАТЕМАТИКА
# =====================
def get_distance(lat1, lon1, lat2, lon2):
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    return math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5

def get_bearing(lat1, lon1, lat2, lon2):
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon_r = math.radians(lon2 - lon1)

    y = math.sin(dlon_r) * math.cos(lat2_r)
    x = (
        math.cos(lat1_r) * math.sin(lat2_r)
        - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon_r)
    )
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def clamp(v, mn, mx):
    return max(mn, min(mx, v))

def set_rc(roll=1500, pitch=1500, throttle=1390, yaw=1500):
    vehicle.channels.overrides = {
        "1": int(roll),
        "2": int(pitch),
        "3": int(throttle),
        "4": int(yaw),
    }

# =====================
# ARM
# =====================
while not vehicle.is_armable:
    time.sleep(1)

vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True

while not vehicle.armed:
    time.sleep(1)

print("ARMED")

# =====================
# ВИСОТА (PID)
# =====================
prev_error = 0

def altitude_control(alt):
    global prev_error

    error = TARGET_ALT - alt

    if abs(error) < 0.6:
        return RC_THROTTLE_HOVER

    d_error = error - prev_error

    throttle = RC_THROTTLE_HOVER + error * 2.8 + d_error * 5
    throttle = clamp(throttle, RC_THROTTLE_MIN, RC_THROTTLE_MAX)

    prev_error = error
    return int(throttle)

# =====================
# ЗЛІТ
# =====================
print("Takeoff...")

while True:
    alt = vehicle.location.global_relative_frame.alt or 0

    if alt >= TARGET_ALT * 0.97:
        break

    if alt < 70:
        throttle = 1450
    elif alt < 90:
        throttle = 1420
    else:
        throttle = 1390

    set_rc(throttle=throttle)
    time.sleep(0.3)

# гасимо інерцію
set_rc(throttle=1280)
time.sleep(1.2)

set_rc(throttle=RC_THROTTLE_HOVER)

# =====================
# ПОЛІТ
# =====================
print("Flying...")

prev_dist = None
stuck_counter = 0

while True:
    current = vehicle.location.global_frame
    alt = vehicle.location.global_relative_frame.alt or 0
    heading = vehicle.heading or 0

    dist = get_distance(current.lat, current.lon, TARGET_LAT, TARGET_LON)
    bearing = get_bearing(current.lat, current.lon, TARGET_LAT, TARGET_LON)
    diff = normalize_angle(bearing - heading)

    print(f"Dist: {dist:.2f} | Alt: {alt:.2f} | Diff: {diff:.2f}")

    # 🔥 точніше підлітаємо
    if dist < 2:
        print("FINAL POSITION LOCK")
        break

    # ---------------------
    # ВИСОТА
    # ---------------------
    throttle = altitude_control(alt)

    if alt > TARGET_ALT + 2:
        throttle = 1318

    # ---------------------
    # YAW
    # ---------------------
    yaw = clamp(RC_YAW_NEUTRAL + diff * 6, YAW_MIN, YAW_MAX)

    # ---------------------
    # РУХ
    # ---------------------
    if abs(diff) > 50:
        pitch = 1500
    elif abs(diff) > 15:
        pitch = PITCH_FORWARD_SLOW
    elif abs(diff) > 7:
        pitch = PITCH_FORWARD_MED
    else:
        pitch = PITCH_FORWARD_FAST
    

    # 🔥 нова логіка підльоту
    if dist < 60:
        pitch = PITCH_FORWARD_MED
    if dist < 50:
        pitch = PITCH_FORWARD_SLOW
    if dist < 40:
        pitch = 1400
    if dist < 30:
        pitch = 1420
    if dist < 20:
        pitch = 1450
    if dist < 10:
        pitch = 1500

    # 🔥 force підліт
    if dist < 13 and abs(diff) < 10:
        pitch = 1330

    # 🔥 якщо завис
    if prev_dist and abs(dist - prev_dist) < 0.25:
        stuck_counter += 1
    else:
        stuck_counter = 0

    if stuck_counter > 4:
        print("STUCK → PUSH HARD")
        pitch = 1260
        stuck_counter = 0

    prev_dist = dist

    set_rc(pitch=pitch, throttle=throttle, yaw=yaw)
    time.sleep(0.25)

# =====================
# ПОСАДКА
# =====================

print("Landing with position hold...")

while True:
    loc = vehicle.location.global_frame
    alt = vehicle.location.global_relative_frame.alt or 0

    print(f"Alt: {alt:.2f}")

    # STOP
    if alt <= 0.3:
        print("Touchdown detected")
        break

    # ---------------------
    # ВИСОТА
    # ---------------------
    if alt > 30:
        throttle = 1300
    elif alt > 20:
        throttle = 1310
    elif alt > 10:
        throttle = 1330
    elif alt > 8:
        throttle = 1350
    elif alt > 6:
        throttle = 1360
    elif alt > 4:
        throttle = 1362
    elif alt > 2:
        throttle = 1364
    else:
        throttle = 1365

    # ---------------------
    # ПОЗИЦІЙНА КОРЕКЦІЯ
    # ---------------------
    dist = get_distance(loc.lat, loc.lon, TARGET_LAT, TARGET_LON)

    # напрямок до точки
    bearing = get_bearing(loc.lat, loc.lon, TARGET_LAT, TARGET_LON)
    heading = vehicle.heading or 0

    diff = normalize_angle(bearing - heading)

    # YAW (дивимось на точку)
    yaw = clamp(1500 - diff * 3, 1400, 1600)

    # ---------------------
    # КОРЕКЦІЯ РУХУ
    # ---------------------
    pitch = 1500
    roll = 1500

    if dist > 0.5:
        # чим ближче — тим слабша корекція
        if alt > 10:
            power = 20
        else:
            power = 10

        # рух вперед/назад
        if abs(diff) < 20:
            pitch = 1500 - power
        elif abs(diff) > 160:
            pitch = 1500 + power

        # боковий drift (дуже важливо)
        if diff > 20:
            roll = 1500 + power
        elif diff < -20:
            roll = 1500 - power

    # ---------------------
    # APPLY
    # ---------------------
    vehicle.channels.overrides = {
        "1": int(roll),
        "2": int(pitch),
        "3": int(throttle),
        "4": int(yaw),
    }

    time.sleep(0.3)

vehicle.channels.overrides = {}
print("DONE")