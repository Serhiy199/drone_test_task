# from dronekit import connect, VehicleMode
# import time
# import math

# # =====================
# # ПІДКЛЮЧЕННЯ
# # =====================
# print("Connecting...")
# vehicle = connect("127.0.0.1:14551", wait_ready=True)

# # =====================
# # ВІТЕР (як у ТЗ)
# # =====================
# vehicle.parameters["SIM_WIND_SPD"] = 3
# vehicle.parameters["SIM_WIND_DIR"] = 30
# vehicle.parameters["SIM_WIND_TURB"] = 2
# vehicle.parameters["SIM_WIND_TURB_FREQ"] = 0.2

# time.sleep(1)
# print("Wind configured")

# # =====================
# # ТОЧКИ
# # =====================
# START_LAT = 50.450739
# START_LON = 30.461242

# TARGET_LAT = 50.443326
# TARGET_LON = 30.448078
# TARGET_ALT = 300.0   # по пораді рекрутера тестуємо на 300м

# # =====================
# # RC НАЛАШТУВАННЯ
# # =====================
# RC_ROLL_NEUTRAL = 1500
# RC_PITCH_NEUTRAL = 1500
# RC_YAW_NEUTRAL = 1500

# RC_THROTTLE_HOVER = 1398
# RC_THROTTLE_MIN = 1200
# RC_THROTTLE_MAX = 1485

# ROLL_MIN = 1320
# ROLL_MAX = 1680
# PITCH_MIN = 1320
# PITCH_MAX = 1680

# FIXED_YAW = 1500  # yaw не чіпаємо

# # =====================
# # PID КОЕФІЦІЄНТИ
# # =====================
# # висота
# ALT_KP = 1.8
# ALT_KD = 2.5

# # позиція (сильніше, ніж було)
# POS_KP = 4.0
# POS_KD = 8.0

# # посадка
# LAND_KP = 4.8
# LAND_KD = 9.0

# # мінімальний поштовх, щоб не зависати у вітрі
# MIN_PUSH = 55

# # межі корекцій
# MAX_POS_CMD = 180
# MAX_LAND_CMD = 120

# # =====================
# # ДОПОМІЖНІ ФУНКЦІЇ
# # =====================
# def clamp(v, mn, mx):
#     return max(mn, min(mx, v))

# def set_rc(roll=1500, pitch=1500, throttle=1390, yaw=1500):
#     vehicle.channels.overrides = {
#         "1": int(roll),      # roll
#         "2": int(pitch),     # pitch
#         "3": int(throttle),  # throttle
#         "4": int(yaw),       # yaw
#     }

# def latlon_to_meters(lat1, lon1, lat2, lon2):
#     d_north = (lat2 - lat1) * 111320.0
#     d_east = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
#     return d_north, d_east

# def distance_m(lat1, lon1, lat2, lon2):
#     dn, de = latlon_to_meters(lat1, lon1, lat2, lon2)
#     return math.sqrt(dn * dn + de * de)

# def world_to_body(north_err, east_err, heading_deg):
#     hdg = math.radians(heading_deg)
#     forward = north_err * math.cos(hdg) + east_err * math.sin(hdg)
#     right = -north_err * math.sin(hdg) + east_err * math.cos(hdg)
#     return forward, right

# def apply_min_push(delta, min_push):
#     if abs(delta) < 1e-6:
#         return 0.0
#     if abs(delta) < min_push:
#         return min_push if delta > 0 else -min_push
#     return delta

# prev_alt_error = 0.0

# def altitude_control(current_alt):
#     global prev_alt_error

#     error = TARGET_ALT - current_alt
#     d_error = error - prev_alt_error

#     throttle = RC_THROTTLE_HOVER + error * ALT_KP + d_error * ALT_KD

#     # якщо перелетіли ціль по висоті - примусово підштовхуємо вниз
#     if current_alt > TARGET_ALT + 8:
#         throttle = 1310
#     elif current_alt > TARGET_ALT + 4:
#         throttle = min(throttle, 1330)

#     throttle = clamp(throttle, RC_THROTTLE_MIN, RC_THROTTLE_MAX)
#     prev_alt_error = error
#     return int(throttle)

# def landing_throttle_control(current_alt):
#     if current_alt > 180:
#         return 1315
#     elif current_alt > 120:
#         return 1325
#     elif current_alt > 80:
#         return 1335
#     elif current_alt > 50:
#         return 1348
#     elif current_alt > 25:
#         return 1358
#     elif current_alt > 12:
#         return 1366
#     elif current_alt > 5:
#         return 1372
#     elif current_alt > 2:
#         return 1378
#     else:
#         return 1384

# def emergency_stop():
#     vehicle.channels.overrides = {}
#     print("SAFE STOP")

# # =====================
# # ПЕРЕВІРКА ГОТОВНОСТІ
# # =====================
# print("Waiting for vehicle to become armable...")
# t0 = time.time()
# while not vehicle.is_armable:
#     if time.time() - t0 > 60:
#         raise RuntimeError("Vehicle is not armable")
#     time.sleep(1)

# print("Waiting for GPS fix...")
# t0 = time.time()
# while vehicle.location.global_frame.lat is None or vehicle.location.global_frame.lon is None:
#     if time.time() - t0 > 60:
#         raise RuntimeError("GPS position is unavailable")
#     time.sleep(1)

# # =====================
# # ARM
# # =====================
# vehicle.mode = VehicleMode("STABILIZE")
# time.sleep(1)

# print("Arming motors...")
# vehicle.armed = True

# t0 = time.time()
# while not vehicle.armed:
#     if time.time() - t0 > 20:
#         raise RuntimeError("Arming failed")
#     time.sleep(0.5)

# print("ARMED")
# time.sleep(2)

# # =====================
# # ОСНОВНИЙ КОД
# # =====================
# try:
#     # =====================
#     # ЗЛІТ
#     # =====================
#     print("Takeoff...")

#     while True:
#         alt = vehicle.location.global_relative_frame.alt or 0.0
#         print(f"Takeoff alt: {alt:.2f}")

#         if alt >= TARGET_ALT * 0.97:
#             break

#         if alt < 100:
#             throttle = 1455
#         elif alt < 220:
#             throttle = 1430
#         else:
#             throttle = 1410

#         set_rc(
#             roll=RC_ROLL_NEUTRAL,
#             pitch=RC_PITCH_NEUTRAL,
#             throttle=throttle,
#             yaw=FIXED_YAW
#         )
#         time.sleep(0.25)

#     # гасимо інерцію після набору висоти
#     set_rc(
#         roll=RC_ROLL_NEUTRAL,
#         pitch=RC_PITCH_NEUTRAL,
#         throttle=1320,
#         yaw=FIXED_YAW
#     )
#     time.sleep(1.0)

#     set_rc(
#         roll=RC_ROLL_NEUTRAL,
#         pitch=RC_PITCH_NEUTRAL,
#         throttle=RC_THROTTLE_HOVER,
#         yaw=FIXED_YAW
#     )
#     time.sleep(1.0)

#     # =====================
#     # ПОЛІТ ДО ТОЧКИ Б
#     # =====================
#     print("Flying to target without yaw rotation...")

#     prev_forward_error = 0.0
#     prev_right_error = 0.0
#     arrival_counter = 0

#     while True:
#         current = vehicle.location.global_frame
#         current_alt = vehicle.location.global_relative_frame.alt or 0.0
#         heading = vehicle.heading or 0.0

#         north_err, east_err = latlon_to_meters(
#             current.lat, current.lon, TARGET_LAT, TARGET_LON
#         )
#         dist = math.sqrt(north_err * north_err + east_err * east_err)

#         forward_err, right_err = world_to_body(north_err, east_err, heading)

#         d_forward = forward_err - prev_forward_error
#         d_right = right_err - prev_right_error

#         # PID по позиції
#         forward_delta = forward_err * POS_KP + d_forward * POS_KD
#         right_delta = right_err * POS_KP + d_right * POS_KD

#         # обмежуємо максимум
#         forward_delta = clamp(forward_delta, -MAX_POS_CMD, MAX_POS_CMD)
#         right_delta = clamp(right_delta, -MAX_POS_CMD, MAX_POS_CMD)

#         # мінімальний поштовх, якщо ще далеко
#         if dist > 8:
#             forward_delta = apply_min_push(forward_delta, MIN_PUSH)
#             right_delta = apply_min_push(right_delta, MIN_PUSH)

#         pitch_cmd = RC_PITCH_NEUTRAL - forward_delta
#         roll_cmd = RC_ROLL_NEUTRAL + right_delta

#         # плавне, але не надто сильне гальмування біля цілі
#         if dist < 60:
#             pitch_cmd = RC_PITCH_NEUTRAL + (pitch_cmd - RC_PITCH_NEUTRAL) * 0.90
#             roll_cmd = RC_ROLL_NEUTRAL + (roll_cmd - RC_ROLL_NEUTRAL) * 0.90

#         if dist < 25:
#             pitch_cmd = RC_PITCH_NEUTRAL + (pitch_cmd - RC_PITCH_NEUTRAL) * 0.80
#             roll_cmd = RC_ROLL_NEUTRAL + (roll_cmd - RC_ROLL_NEUTRAL) * 0.80

#         if dist < 10:
#             pitch_cmd = RC_PITCH_NEUTRAL + (pitch_cmd - RC_PITCH_NEUTRAL) * 0.65
#             roll_cmd = RC_ROLL_NEUTRAL + (roll_cmd - RC_ROLL_NEUTRAL) * 0.65

#         pitch_cmd = clamp(pitch_cmd, PITCH_MIN, PITCH_MAX)
#         roll_cmd = clamp(roll_cmd, ROLL_MIN, ROLL_MAX)

#         throttle = altitude_control(current_alt)

#         print(
#             f"Dist: {dist:.2f} m | Alt: {current_alt:.2f} m | "
#             f"N_err: {north_err:.2f} | E_err: {east_err:.2f} | "
#             f"F_err: {forward_err:.2f} | R_err: {right_err:.2f} | "
#             f"Pitch: {pitch_cmd:.1f} | Roll: {roll_cmd:.1f}"
#         )

#         # умова входу в зону посадки
#         if dist < 5.0:
#             arrival_counter += 1
#         else:
#             arrival_counter = 0

#         if arrival_counter >= 8:
#             print("Reached target area")
#             break

#         set_rc(
#             roll=roll_cmd,
#             pitch=pitch_cmd,
#             throttle=throttle,
#             yaw=FIXED_YAW
#         )

#         prev_forward_error = forward_err
#         prev_right_error = right_err

#         time.sleep(0.15)

#     # =====================
#     # СТАБІЛІЗАЦІЯ НАД ТОЧКОЮ
#     # =====================
#     print("Stabilizing over target...")

#     for _ in range(25):
#         current = vehicle.location.global_frame
#         current_alt = vehicle.location.global_relative_frame.alt or 0.0
#         heading = vehicle.heading or 0.0

#         north_err, east_err = latlon_to_meters(
#             current.lat, current.lon, TARGET_LAT, TARGET_LON
#         )
#         forward_err, right_err = world_to_body(north_err, east_err, heading)

#         d_forward = forward_err - prev_forward_error
#         d_right = right_err - prev_right_error

#         forward_delta = forward_err * LAND_KP + d_forward * LAND_KD
#         right_delta = right_err * LAND_KP + d_right * LAND_KD

#         forward_delta = clamp(forward_delta, -MAX_LAND_CMD, MAX_LAND_CMD)
#         right_delta = clamp(right_delta, -MAX_LAND_CMD, MAX_LAND_CMD)

#         if abs(forward_err) > 1.0:
#             forward_delta = apply_min_push(forward_delta, 25)
#         if abs(right_err) > 1.0:
#             right_delta = apply_min_push(right_delta, 25)

#         pitch_cmd = RC_PITCH_NEUTRAL - forward_delta
#         roll_cmd = RC_ROLL_NEUTRAL + right_delta

#         pitch_cmd = clamp(pitch_cmd, 1400, 1600)
#         roll_cmd = clamp(roll_cmd, 1400, 1600)

#         throttle = altitude_control(current_alt)

#         set_rc(
#             roll=roll_cmd,
#             pitch=pitch_cmd,
#             throttle=throttle,
#             yaw=FIXED_YAW
#         )

#         prev_forward_error = forward_err
#         prev_right_error = right_err

#         time.sleep(0.15)

#     # =====================
#     # ПОСАДКА В ТОЧКУ Б
#     # =====================
#     print("Landing with PID position hold...")

#     while True:
#         current = vehicle.location.global_frame
#         current_alt = vehicle.location.global_relative_frame.alt or 0.0
#         heading = vehicle.heading or 0.0

#         north_err, east_err = latlon_to_meters(
#             current.lat, current.lon, TARGET_LAT, TARGET_LON
#         )
#         dist = math.sqrt(north_err * north_err + east_err * east_err)

#         forward_err, right_err = world_to_body(north_err, east_err, heading)

#         d_forward = forward_err - prev_forward_error
#         d_right = right_err - prev_right_error

#         forward_delta = forward_err * LAND_KP + d_forward * LAND_KD
#         right_delta = right_err * LAND_KP + d_right * LAND_KD

#         forward_delta = clamp(forward_delta, -MAX_LAND_CMD, MAX_LAND_CMD)
#         right_delta = clamp(right_delta, -MAX_LAND_CMD, MAX_LAND_CMD)

#         # нижче — м'якше, але не в нуль
#         gain_scale = 1.0
#         if current_alt < 30:
#             gain_scale = 0.85
#         if current_alt < 10:
#             gain_scale = 0.70
#         if current_alt < 3:
#             gain_scale = 0.55

#         forward_delta *= gain_scale
#         right_delta *= gain_scale

#         if dist > 1.5:
#             if abs(forward_delta) > 1.0:
#                 forward_delta = apply_min_push(forward_delta, 18)
#             if abs(right_delta) > 1.0:
#                 right_delta = apply_min_push(right_delta, 18)

#         pitch_cmd = RC_PITCH_NEUTRAL - forward_delta
#         roll_cmd = RC_ROLL_NEUTRAL + right_delta

#         pitch_cmd = clamp(pitch_cmd, 1430, 1570)
#         roll_cmd = clamp(roll_cmd, 1430, 1570)

#         throttle = landing_throttle_control(current_alt)

#         print(
#             f"Landing | Dist: {dist:.2f} m | Alt: {current_alt:.2f} m | "
#             f"F_err: {forward_err:.2f} | R_err: {right_err:.2f} | "
#             f"Pitch: {pitch_cmd:.1f} | Roll: {roll_cmd:.1f}"
#         )

#         if current_alt <= 0.35:
#             print("Touchdown detected")
#             break

#         set_rc(
#             roll=roll_cmd,
#             pitch=pitch_cmd,
#             throttle=throttle,
#             yaw=FIXED_YAW
#         )

#         prev_forward_error = forward_err
#         prev_right_error = right_err

#         time.sleep(0.15)

# except Exception as e:
#     print("ERROR:", e)

# finally:
#     emergency_stop()
#     print("DONE")




from dronekit import connect, VehicleMode
import time
import math

print("Connecting...")
vehicle = connect("127.0.0.1:14550", wait_ready=True)

# =====================
# ВІТЕРSIM_WIND
# # =====================

# time.sleep(3)
# vehicle.parameters["SIM_WIND_SPD"] = 46
# # vehicle.parameters["SIM_WIND_DIR"] = 30
# vehicle.parameters["SIM_WIND_DIR"] = 90
# # vehicle.parameters["SIM_WIND_TURB"] = 2
# vehicle.parameters["SIM_WIND_TURB"] = 6
# vehicle.parameters["SIM_WIND_TURB_FREQ"] = 0.2

# =====================
# ТОЧКИ
# =====================
TARGET_LAT = 50.443326
TARGET_LON = 30.448078
TARGET_ALT = 300.0



# =====================
# RC / БАЗОВІ НАЛАШТУВАННЯ
# =====================
RC_NEUTRAL = 1500
YAW = 1500                   # yaw не змінюємо весь політ
HOVER_THROTTLE = 1390        # підбери за симуляцією
MAX_RC_STEP_PER_SEC = 220    # плавність зміни RC-команд
DT = 0.1

ROLL_MIN = 1360
ROLL_MAX = 1640
PITCH_MIN = 1360
PITCH_MAX = 1640
THROTTLE_MIN = 1300
THROTTLE_MAX = 1555

# =====================
# ПАРАМЕТРИ ПОЛЬОТУ
# =====================
MAX_SPEED = 10.0             # максимум 10 м/с
MIN_APPROACH_SPEED = 5     # мінімум біля цілі, щоб вітер не "зависив"
SLOW_RADIUS = 80.0           # починаємо гальмування за 80 м
LAND_START_DIST = 3.0        # якщо ближче, починаємо посадку
WIND_GAIN = 2.5              # компенсація зносу по швидкості
SPEED_GAIN = 24.0            # наскільки агресивно розганяємось
FINAL_BRAKE_RADIUS = 12.0    # зона більш точного дотискання

# =====================
# PID
# =====================
class PID:
    def __init__(self, kp, ki, kd, i_limit=None, out_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, error, dt, derivative_override=None):
        if dt <= 0:
            dt = 1e-3

        self.integral += error * dt
        if self.i_limit is not None:
            self.integral = clamp(self.integral, -self.i_limit, self.i_limit)

        if derivative_override is not None:
            derivative = derivative_override
        else:
            if self.first:
                derivative = 0.0
                self.first = False
            else:
                derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.out_limit is not None:
            output = clamp(output, -self.out_limit, self.out_limit)

        self.prev_error = error
        return output


def clamp(v, mn, mx):
    return max(mn, min(mx, v))


def get_distance(lat1, lon1, lat2, lon2):
    """
    dn  - зміщення до цілі по North (м)
    de  - зміщення до цілі по East  (м)
    dist - відстань до цілі (м)
    """
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    d_lat = (lat2 - lat1) * 111320.0
    d_lon = (lon2 - lon1) * 111320.0 * math.cos(mean_lat)
    dist = math.sqrt(d_lat * d_lat + d_lon * d_lon)
    return d_lat, d_lon, dist


def earth_to_body(cmd_n, cmd_e, heading_deg):
    """
    Перетворення з Earth frame (North/East) у Body frame (forward/right).
    Yaw не змінюємо, але треба правильно переводити команди для roll/pitch.
    """
    h = math.radians(heading_deg or 0.0)
    forward = math.cos(h) * cmd_n + math.sin(h) * cmd_e
    right = -math.sin(h) * cmd_n + math.cos(h) * cmd_e
    return forward, right


last_roll = RC_NEUTRAL
last_pitch = RC_NEUTRAL
last_throttle = HOVER_THROTTLE


def slew(current, target, max_delta):
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


def set_rc(roll, pitch, throttle, dt):
    global last_roll, last_pitch, last_throttle

    max_step = MAX_RC_STEP_PER_SEC * dt

    roll = slew(last_roll, roll, max_step)
    pitch = slew(last_pitch, pitch, max_step)
    throttle = slew(last_throttle, throttle, max_step)

    roll = int(clamp(roll, ROLL_MIN, ROLL_MAX))
    pitch = int(clamp(pitch, PITCH_MIN, PITCH_MAX))
    throttle = int(clamp(throttle, THROTTLE_MIN, THROTTLE_MAX))

    vehicle.channels.overrides = {
        "1": roll,       # roll
        "2": pitch,      # pitch
        "3": throttle,   # throttle
        "4": YAW,        # yaw fixed
    }

    last_roll = roll
    last_pitch = pitch
    last_throttle = throttle


# =====================
# PID КОНТРОЛЕРИ
# =====================
# Горизонтальне утримання / дотискання до точки
pid_n = PID(kp=0.45, ki=0.020, kd=2.40, i_limit=80, out_limit=140)
pid_e = PID(kp=0.45, ki=0.020, kd=2.40, i_limit=80, out_limit=140)

# Висота
pid_alt = PID(kp=5.8, ki=0.06, kd=6.5, i_limit=60, out_limit=110)

# =====================
# ARM
# =====================
print("Waiting until armable...")
while not vehicle.is_armable:
    time.sleep(1)

vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True

while not vehicle.armed:
    time.sleep(0.5)

print("ARMED")

# =====================
# ЗЛІТ
# =====================
print("Takeoff...")

takeoff_target_alt = 0.0
takeoff_rate = 4.5

prev_alt = vehicle.location.global_relative_frame.alt or 0.0
prev_time = time.time()

while True:
    now = time.time()
    dt = now - prev_time
    if dt <= 0:
        dt = DT
    prev_time = now

    alt = vehicle.location.global_relative_frame.alt or 0.0
    alt_rate = (alt - prev_alt) / dt
    prev_alt = alt

    takeoff_target_alt = min(TARGET_ALT, takeoff_target_alt + takeoff_rate * dt)

    alt_error = takeoff_target_alt - alt
    alt_derivative = takeoff_rate - alt_rate if takeoff_target_alt < TARGET_ALT else -alt_rate

    throttle_cmd = HOVER_THROTTLE + pid_alt.update(
        error=alt_error,
        dt=dt,
        derivative_override=alt_derivative
    )

    set_rc(RC_NEUTRAL, RC_NEUTRAL, throttle_cmd, dt)

    print(f"TAKEOFF | alt={alt:.1f} target={takeoff_target_alt:.1f} thr={int(throttle_cmd)}")

    if alt >= TARGET_ALT - 2.0 and abs(alt_rate) < 1.0:
        break

    time.sleep(DT)

print("Reached altitude")

pid_n.reset()
pid_e.reset()

# =====================
# ПОЛІТ ДО ТОЧКИ B
# =====================
print("Flying...")

prev_time = time.time()
prev_alt = vehicle.location.global_relative_frame.alt or 0.0

while True:
    now = time.time()
    dt = now - prev_time
    if dt <= 0:
        dt = DT
    prev_time = now

    loc = vehicle.location.global_frame
    alt = vehicle.location.global_relative_frame.alt or 0.0
    heading = vehicle.heading or 0.0

    dn, de, dist = get_distance(loc.lat, loc.lon, TARGET_LAT, TARGET_LON)

    vel = vehicle.velocity or [0.0, 0.0, 0.0]
    vn = vel[0] or 0.0
    ve = vel[1] or 0.0
    speed_xy = math.sqrt(vn * vn + ve * ve)

    alt_rate = (alt - prev_alt) / dt
    prev_alt = alt

    # Уже достатньо близько — переходимо до посадки,
    # не чекаємо "ідеального нуля", бо вітер може тримати збоку.
    if dist < LAND_START_DIST:
        print("Close enough to target, switching to landing")
        break

    # ===== ЦІЛЬОВА ШВИДКІСТЬ =====
    if dist > SLOW_RADIUS:
        target_speed = MAX_SPEED
    else:
        scaled_speed = MAX_SPEED * (dist / SLOW_RADIUS)
        target_speed = max(MIN_APPROACH_SPEED, scaled_speed)

    # ===== НАПРЯМОК ДО ЦІЛІ =====
    if dist > 0.01:
        dir_n = dn / dist
        dir_e = de / dist
    else:
        dir_n = 0.0
        dir_e = 0.0

    # ===== ПОМИЛКА ПО ШВИДКОСТІ =====
    speed_error = target_speed - speed_xy

    # ===== PID-КОРЕКЦІЯ ПО ПОЗИЦІЇ =====
    # derivative error приблизно = -current_velocity
    pos_corr_n = pid_n.update(error=dn, dt=dt, derivative_override=-vn)
    pos_corr_e = pid_e.update(error=de, dt=dt, derivative_override=-ve)

    # ===== БАЗОВА КОМАНДА РУХУ =====
    cmd_n = dir_n * speed_error * SPEED_GAIN + pos_corr_n * 0.7
    cmd_e = dir_e * speed_error * SPEED_GAIN + pos_corr_e * 0.7

    # ===== АНТИ-ВІТЕР / КОМПЕНСАЦІЯ ЗНОСУ =====
    # Якщо вітер або інерція зносять — підсилюємо компенсацію по швидкості.
    # cmd_n += -vn * WIND_GAIN
    # cmd_e += -ve * WIND_GAIN

    # ===== ДОДАТКОВЕ ДОТИСКАННЯ БІЛЯ ЦІЛІ =====
    if dist < FINAL_BRAKE_RADIUS:
        cmd_n += dn * 0.9
        cmd_e += de * 0.9

    # ===== ОБМЕЖЕННЯ =====
    max_horiz_cmd = 180
    if dist < 40:
        max_horiz_cmd = 170
    if dist < 15:
        max_horiz_cmd = 150
    if dist < 8:
        max_horiz_cmd = 130
    if dist < 4:
        max_horiz_cmd = 110

    cmd_n = clamp(cmd_n, -max_horiz_cmd, max_horiz_cmd)
    cmd_e = clamp(cmd_e, -max_horiz_cmd, max_horiz_cmd)

    # ===== ПЕРЕТВОРЕННЯ EARTH -> BODY =====
    forward_cmd, right_cmd = earth_to_body(cmd_n, cmd_e, heading)

    pitch = RC_NEUTRAL - forward_cmd
    roll = RC_NEUTRAL + right_cmd

    # ===== ВИСОТА =====
    alt_error = TARGET_ALT - alt
    throttle = HOVER_THROTTLE + pid_alt.update(
        error=alt_error,
        dt=dt,
        derivative_override=-alt_rate
    )

    set_rc(roll, pitch, throttle, dt)

    print(
        f"FLY | dist={dist:6.2f}m | dn={dn:7.2f} de={de:7.2f} | "
        f"v={speed_xy:4.2f}/{target_speed:4.2f} | "
        f"cmdN={cmd_n:6.1f} cmdE={cmd_e:6.1f} | "
        f"roll={int(roll)} pitch={int(pitch)} thr={int(throttle)}"
    )

    time.sleep(DT)

# =====================
# КОРОТКА СТАБІЛІЗАЦІЯ НАД ТОЧКОЮ
# =====================
print("Stabilizing over target...")

pid_n.reset()
pid_e.reset()

prev_time = time.time()
prev_alt = vehicle.location.global_relative_frame.alt or 0.0
stable_start = time.time()

while time.time() - stable_start < 2.5:
    now = time.time()
    dt = now - prev_time
    if dt <= 0:
        dt = DT
    prev_time = now

    loc = vehicle.location.global_frame
    alt = vehicle.location.global_relative_frame.alt or 0.0
    heading = vehicle.heading or 0.0

    dn, de, dist = get_distance(loc.lat, loc.lon, TARGET_LAT, TARGET_LON)

    vel = vehicle.velocity or [0.0, 0.0, 0.0]
    vn = vel[0] or 0.0
    ve = vel[1] or 0.0

    alt_rate = (alt - prev_alt) / dt
    prev_alt = alt

    cmd_n = pid_n.update(error=dn, dt=dt, derivative_override=-vn) + (-vn * 1.2)
    cmd_e = pid_e.update(error=de, dt=dt, derivative_override=-ve) + (-ve * 1.2)

    cmd_n = clamp(cmd_n, -55, 55)
    cmd_e = clamp(cmd_e, -55, 55)

    forward_cmd, right_cmd = earth_to_body(cmd_n, cmd_e, heading)

    pitch = RC_NEUTRAL - forward_cmd
    roll = RC_NEUTRAL + right_cmd

    alt_error = TARGET_ALT - alt
    throttle = HOVER_THROTTLE + pid_alt.update(
        error=alt_error,
        dt=dt,
        derivative_override=-alt_rate
    )

    set_rc(roll, pitch, throttle, dt)

    print(f"STAB | dist={dist:.2f}m | alt={alt:.1f} | dn={dn:.2f} de={de:.2f}")

    time.sleep(DT)

# =====================
# ПОСАДКА З PID + АНТИ-ВІТЕР
# =====================
print("Landing...")

landing_target_alt = TARGET_ALT
prev_time = time.time()
prev_alt = vehicle.location.global_relative_frame.alt or 0.0

while True:
    now = time.time()
    dt = now - prev_time
    if dt <= 0:
        dt = DT
    prev_time = now

    loc = vehicle.location.global_frame
    alt = vehicle.location.global_relative_frame.alt or 0.0

    if alt < 0.4:
        break

    dn, de, dist = get_distance(loc.lat, loc.lon, TARGET_LAT, TARGET_LON)

    vel = vehicle.velocity or [0.0, 0.0, 0.0]
    vn = vel[0]
    ve = vel[1]

    alt_rate = (alt - prev_alt) / dt
    prev_alt = alt

    # ===== ГОРИЗОНТАЛЬ =====
    if alt > 10:
        Kp = 6
        Kv = 4
    else:
        Kp = 14   # 🔥 сильніше тримає точку біля землі
        Kv = 8    # 🔥 гасить вітер

    pitch = 1500 - dn * Kp - vn * Kv
    roll  = 1500 + de * Kp - ve * Kv

    # 🔥 важливо — розширюємо діапазон
    pitch = clamp(pitch, 1400, 1600)
    roll  = clamp(roll, 1400, 1600)

    # ===== ШВИДКІСТЬ СПУСКУ =====
    if alt > 50:
        landing_rate = 1.8
    elif alt > 20:
        landing_rate = 1.2
    elif alt > 8:
        landing_rate = 0.8
    elif alt > 3:
        landing_rate = 0.5
    else:
        landing_rate = 0.3

    # 🔥 ВАЖЛИВО: НЕ ГАЛЬМУЄМО СПУСК ЯКЩО НЕ НАД ТОЧКОЮ
    if dist > 2.5:
        landing_rate *= 0.7   # трохи повільніше, але не 0!

    # ===== ТРАЄКТОРІЯ ВИСОТИ =====
    landing_target_alt = max(0, landing_target_alt - landing_rate * dt)

    alt_error = landing_target_alt - alt

    # throttle = HOVER_THROTTLE + alt_error * 6 - alt_rate * 3
    if alt > 4:
    # нормальна PID логіка
        throttle = HOVER_THROTTLE + alt_error * 6 - alt_rate * 3
    else:
    # 🔥 ЖОРСТКИЙ СПУСК
        throttle = 1340

    # 🔥 ДОЖИМ НА МАЛІЙ ВИСОТІ
    if alt < 5:
        throttle -= 10
    if alt < 2:
        throttle -= 8

    throttle = clamp(throttle, 1300, 1500)

    # set_rc(roll, pitch, throttle)
    set_rc(roll, pitch, throttle, dt)

    print(f"LAND | alt={alt:.2f} | dist={dist:.2f}")

    time.sleep(DT)

# =====================
# ЗАВЕРШЕННЯ
# =====================
vehicle.channels.overrides = {}
print("DONE")
