from math import e, sqrt, sin, cos, atan, degrees, pi
from decimal import Decimal, getcontext
import csv

from matplotlib import pyplot as plt

getcontext().prec = 3334

V0 = 0  # начальная скорость
delta_t = 0.001  # дискретное время
m0 = 45292
m_ty = 4200
m_y = 5840
m_c = 11050
A0 = 10
A = 3
G = 6.67e-11
F_tyag = 989040
F_tyagy = 626430
F_tyagc = 326610
I = 283
g = 9.81
R_sp = 287.053
p0 = 1.225
R = 600000
H = 5600
C = 0.35
a0 = 0
b0 = 0
r0 = R
h = r0 - R
p = p0 * e ** (-(h / H))
M = 5.292 * 10 ** 22
t = Decimal('0')
mass_changed = False
tyag_changed = False
T = 21549.425
V_x0 = 2 * 3.1415 * R / T
V_y0 = 0
a = 0
b = 0
r_x0 = 0
r_y0 = r = R
F_pr = 0
delta = Decimal('0.001')
disabled = False
enabled = False
angle = False
change_mass = True
blocked = False

time = []
mass = []
speed = []
height = []

time_ksp = []
mass_ksp = []
speed_ksp = []
height_ksp = []

max_r, max_t = 0, 0

while True:
    V_otn = 2 * 3.1415 * r / T
    q = 1 / 2 * p * ((V_x0 - V_otn * cos(a0)) ** 2 + (V_y0 + V_otn * sin(a0)) ** 2)

    if q > 60_000 and not tyag_changed:
        F_pr = F_tyag
        F_tyag = 0
        tyag_changed = True
        change_mass = False

    if q <= 60_000 and tyag_changed:
        F_tyag = F_pr
        tyag_changed = False
        change_mass = True

    F_tyaj = (G * m0 * M) / (r ** 2)
    F_sopr = C * p * A0 * ((V_x0 - V_otn * cos(a0)) ** 2 + (V_y0 + V_otn * sin(a0)) ** 2)
    V_x = V_x0 + (F_tyag * sin(b0) - F_sopr * sin(b0) - F_tyaj * sin(a0)) * delta_t / m0
    V_y = V_y0 + (F_tyag * cos(b0) - F_sopr * cos(b0) - F_tyaj * cos(a0)) * delta_t / m0
    r_x = r_x0 + V_x * delta_t
    r_y = r_y0 + V_y * delta_t
    r = sqrt(r_x ** 2 + r_y ** 2)
    if r > 670000:
        F_sopr = 0
    V = sqrt(V_x ** 2 + V_y ** 2)
    h = r - R
    p = p0 * e ** (-(h / H))

    if r_y > 0:
        a = atan(r_x / r_y)
    else:
        a = 3.14 + atan(r_x / r_y)

    if not blocked:
        fi = pi + a - b

    if 89 <= degrees(fi) <= 91:
        blocked = True
        b = pi + a - fi

    if not blocked:
        if V_y + V_otn * sin(a) > 0:
            if degrees(atan((V_x - V_otn * cos(a)) / (V_y + V_otn * sin(a)))) <= 3 and angle:
                b = 3.14 / 60
            elif q > 600 and degrees(atan((V_x - V_otn * cos(a)) / (V_y + V_otn * sin(a)))) > 3 and angle:
                b = atan((V_x - V_otn * cos(a)) / (V_y + V_otn * sin(a)))
            elif q < 600 and degrees(atan((V_x - V_otn * cos(a)) / (V_y + V_otn * sin(a)))) > 3 and p > p0 / 2 and angle:
                b = atan((V_x - V_otn * cos(a)) / (V_y + V_otn * sin(a)))
            elif q < 600 and degrees(atan(V_x / V_y)) > 3 and p < p0 / 2 and angle:
                b = atan(V_x / V_y)
            else:
                ...
        else:
            b = 3.14 + atan((V_x - V_otn * cos(a)) / (V_y + V_otn * sin(a)))

    if t >= 50 and not mass_changed:
        m0 = m0 - 3120
        F_tyag = F_tyagc
        A0 = A
        mass_changed = True

    if change_mass:
        if not mass_changed:
            m = m0 - (F_tyag * delta_t) / (I * g)
        else:
            m = m0 - (F_tyagc * delta_t) / (I * g)

    a_0 = G * M * r / (2 * G * M - r * V ** 2)
    r_a = a_0 * (1 + sqrt(1 - (r * V * sin(a - atan(V_x / V_y))) ** 2 / (G * M * a_0)))

    if r_a >= 754.1 * 1000 and not disabled:
        F_tyag, F_prev = 0, F_tyag
        disabled = True
        change_mass = False

    if r_a - 2 <= r and not enabled and disabled:
        F_tyag = F_prev
        enabled = True
        change_mass = True

    if 2106.5 <= V:
        break

    V_x0, V_y0, r_x0, r_y0, m0 = V_x, V_y, r_x, r_y, m
    a0, b0 = a, b
    t += delta

    if V_y >= 150. and b0 < pi / 60 and not angle:
        b0 = 3.14 / 60
        angle = True

    time.append(t)
    mass.append(m0)
    speed.append(V)
    height.append(h)


# read data from ksp log
with open('flight_log_20251218_201226.csv', 'r') as file:
    csv_reader = csv.reader(file)
    for row in list(csv_reader)[1:]:
        time_ksp.append(float(row[0]))
        height_ksp.append(float(row[1]) - 78.6)
        mass_ksp.append(float(row[2]))
        speed_ksp.append(float(row[4]))


# mass
plt.plot(time, mass, color='blue', label='Мат. модель')
plt.plot(time_ksp, mass_ksp, color='green', label='KSP')
plt.title('Масса')
plt.xlabel('Время (секунды)')
plt.ylabel('Масса (кг)')
plt.legend()
plt.savefig('mass_ideal.png')

plt.close()

# speed
plt.plot(time, speed, color='blue', label='Мат. модель')
plt.plot(time_ksp, speed_ksp, color='green', label='KSP')
plt.title('Скорость')
plt.xlabel('Время (секунды)')
plt.ylabel('Скорость (м/с)')
plt.legend()
plt.savefig('speed_ideal.png')

plt.close()

# height
plt.plot(time, height, color='blue', label='Мат. модель')
plt.plot(time_ksp, height_ksp, color='green', label='KSP')
plt.title('Высота')
plt.xlabel('Время (секунды)')
plt.ylabel('Высота (метры)')
plt.legend()
plt.savefig('height_ideal.png')
