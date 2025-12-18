import krpc
import time
import csv
from datetime import datetime
import os

conn = krpc.connect()
space_center = conn.space_center
vessel = space_center.active_vessel
orbit = vessel.orbit
control = vessel.control
ap = vessel.auto_pilot

TARGET_APOAPSIS = 154000
TARGET_PERIAPSIS = 80000
TARGET_APOAPSIS_96 = TARGET_APOAPSIS * 0.96
LAUNCH_AZIMUTH = 90

results_dir = "flight_logs"
if not os.path.exists(results_dir):
    os.makedirs(results_dir)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = os.path.join(results_dir, f"flight_log_{timestamp}.csv")

csv_file = open(csv_filename, 'w', newline='', encoding='utf-8')
csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    'Время от начала полета (секунды)',
    'Высота над уровнем моря (метры)',
    'Масса корабля (тонны)',
    'Тангаж / угол наклона (градусы)',
    'Орбитальная скорость (м/с)',
    'Скорость относительно поверхности (м/с)',
    'Общая скорость (м/с)',
    'Апогейное расстояние (метры)'
])


control.sas = False
ap.reference_frame = vessel.surface_reference_frame
ap.target_pitch_and_heading(90, LAUNCH_AZIMUTH)
ap.engage()
control.throttle = 1.0

phase = 0
turn_complete = False
sas_engaged = False
circularize_started = False
start_time = time.time()
boosters_dropped = False
last_log_time = start_time

altitude_stream = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
pitch_stream = conn.add_stream(getattr, vessel.flight(), 'pitch')
apoapsis_stream = conn.add_stream(getattr, orbit, 'apoapsis_altitude')
periapsis_stream = conn.add_stream(getattr, orbit, 'periapsis_altitude')
time_to_apoapsis_stream = conn.add_stream(getattr, orbit, 'time_to_apoapsis')

try:
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time

        if current_time - last_log_time >= 0.1:
            altitude = altitude_stream()
            pitch = pitch_stream()
            mass = vessel.mass
            apoapsis = apoapsis_stream()
            flight_info = vessel.flight()
            speed_orbital = vessel.orbit.speed
            speed_surface = flight_info.speed
            speed_total = flight_info.speed

            csv_writer.writerow([
                f"{elapsed_time:.2f}",
                f"{altitude:.1f}",
                f"{mass:.2f}",
                f"{pitch:.1f}",
                f"{speed_orbital:.1f}",
                f"{speed_surface:.1f}",
                f"{speed_total:.1f}",
                f"{apoapsis:.1f}"
            ])

            last_log_time = current_time

        if not boosters_dropped and elapsed_time >= 50:
            vessel.control.activate_next_stage()
            boosters_dropped = True

        if phase == 0:
            altitude = altitude_stream()
            if not turn_complete:
                if altitude < 45000 and altitude > 1000:
                    turn_ratio = min((altitude - 1000) / 44000, 1.0)
                    target_pitch = 90 * (1 - turn_ratio)
                    ap.target_pitch_and_heading(target_pitch, LAUNCH_AZIMUTH)
                elif altitude >= 45000:
                    ap.target_pitch_and_heading(0, LAUNCH_AZIMUTH)
                    turn_complete = True

            if apoapsis_stream() >= TARGET_APOAPSIS_96:
                for i in range(3, 0, -1):
                    control.throttle = i * 0.3
                    time.sleep(0.3)

                control.throttle = 0.0
                ap.disengage()
                time.sleep(0.1)
                control.sas = True
                time.sleep(0.1)
                control.sas_mode = space_center.SASMode.prograde
                time.sleep(0.1)
                sas_engaged = True
                phase = 1

        elif phase == 1:
            time_to_ap = time_to_apoapsis_stream()
            if int(time.time()) % 3 == 0 and not circularize_started:
                if time_to_ap < 10:
                    control.throttle = 0.3
                    circularize_started = True
                    phase = 2

        elif phase == 2:
            current_periapsis = periapsis_stream()
            time_to_ap = time_to_apoapsis_stream()
            error = TARGET_PERIAPSIS - current_periapsis

            if error > 0:
                if time_to_ap > 1:
                    throttle_factor = min(1.0, error / 40000)
                    control.throttle = 0.25 + throttle_factor * 0.25
                else:
                    control.throttle = min(0.35, error / 15000)
            else:
                control.throttle = 0.0

            if current_periapsis >= TARGET_PERIAPSIS * 0.96:
                control.throttle = 0.0
                flight_info = vessel.flight()
                csv_writer.writerow([
                    f"{elapsed_time:.2f}",
                    f"{altitude_stream():.1f}",
                    f"{vessel.mass:.2f}",
                    f"{pitch_stream():.1f}",
                    f"{vessel.orbit.speed:.1f}",
                    f"{flight_info.speed:.1f}",
                    f"{flight_info.speed:.1f}",
                    f"{apoapsis_stream():.1f}"
                ])
                break

        time.sleep(0.01)

except KeyboardInterrupt:
    control.throttle = 0.0
    ap.disengage()
    control.sas = False

except Exception as e:
    control.throttle = 0.0
    ap.disengage()
    control.sas = False

finally:
    csv_file.close()
