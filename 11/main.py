"""
Simulate an Inverted Pundulum
Author: Kazumichi INOUE <kazumichiinoue@mail.saitama-u.ac.jp>
Created: 2024/11/03
"""
import serial
import time
import cv2
import numpy as np
import copy
import casadi 
import math

g = 9.81
M = 1
m = 0.2
l = 2

Q = casadi.diag([2.5, 10, 0.01, 0.01])
Q_f = casadi.diag([2.5, 10, 0.01, 0.01])
R = casadi.diag([0.1])

K = 30
T = 1
dt = T/K

x_lb = [-np.inf, -np.inf, -np.inf, -np.inf]
x_ub = [ np.inf,  np.inf,  np.inf,  np.inf]
u_lb = [-15]
u_ub = [ 15]

nu = 1
nx = 4

x_ref = casadi.DM([0, 0, 0, 0])
u_ref = casadi.DM([0])

total = nx*(K+1) + nu*K

def make_f():
    states = casadi.SX.sym("states", nx)
    ctrls = casadi.SX.sym("ctrls", nu)

    x = states[0]
    theta = states[1]
    x_dot = states[2]
    theta_dot = states[3]
    u = ctrls[0]

    sin = casadi.sin(theta)
    cos = casadi.cos(theta)
    det = M + m*sin**2

    # 抵抗項の定数
    c_theta = 0.5  # theta_dot に対する抵抗の係数
    c_x = 0.5      # x_dot に対する抵抗の係数

    x_ddot = (-m * l * sin * theta_dot**2 + m * g * sin * cos + u - c_x * x_dot) / det
    theta_ddot = (-m * l * sin * cos * theta_dot**2 + (M + m) * g * sin + u * cos - c_theta * theta_dot) / (l * det)

    states_dot = casadi.vertcat(x_dot, theta_dot, x_ddot, theta_ddot)
    
    f = casadi.Function("f", [states, ctrls], [states_dot], ["x", "u"], ["x_dot"])

    return f

def make_RK4():
    states = casadi.SX.sym("states", nx)
    ctrls = casadi.SX.sym("ctrls", nu)

    f = make_f()
    
    r1 = f(x=states,         u=ctrls)["x_dot"]
    r2 = f(x=states+dt*r1/2, u=ctrls)["x_dot"]
    r3 = f(x=states+dt*r2/2, u=ctrls)["x_dot"]
    r4 = f(x=states+dt*r3,   u=ctrls)["x_dot"]

    states_next = states + dt*(r1 + 2*r2 + 2*r3 + r4)/6

    RK4 = casadi.Function("RK4", [states, ctrls], [states_next], ["x", "u"], ["x_next"])

    return RK4

def make_integrator():
    states = casadi.SX.sym("states", nx)
    ctrls = casadi.SX.sym("ctrls", nu)

    f = make_f()
    ode = f(x=states, u=ctrls)["x_dot"]
    dae = {"x":states, "p":ctrls, "ode":ode}

    I = casadi.integrator("I", "cvodes", dae, 0, dt)
    return I
    
# シリアルポートの設定
ser = serial.Serial('/dev/cu.usbmodem2102', 115200)

def send_command_list(cmd):
    ser.write(f"{cmd}\n".encode())  
    #print(f"Sent: {cmd} ---> ", end="")
    response = ser.readline().decode('utf-8')
    response = " ".join(response.splitlines())
    #print(f"Recv: {response}")
    return response

height, width = 400, 1500  # Window SIZE
csize = 0.0125/1
img_org = np.full((height, width, 3), (128, 128, 128), dtype=np.uint8)  # RGB mid-gray
cv2.line(img_org, (0, height//2), (width, height//2), (0, 0, 0), 2)
cv2.line(img_org, (width//2, height//2 - 10), (width//2, height//2+10), (0, 0, 0), 2)
for i in range(8):
    shift = int(i/csize)
    cv2.line(img_org, (width//2 + shift, height//2 - 10), (width//2 + shift, height//2+10), (0, 0, 0), 1)
    cv2.line(img_org, (width//2 - shift, height//2 - 10), (width//2 - shift, height//2+10), (0, 0, 0), 1)


length = l/csize
rx = 0
x_init = casadi.DM([0, 0, 0, 0])
u_i = casadi.DM([0])
I = make_integrator()
x_current = x_init

while True:
    cmd = "R"
    res = send_command_list(cmd)
    if res[0] == 'S': break
    elif res[0] == 'C':
        x_current = x_init
        continue
        
    acc_x = float(res[1:])/30
        
    if acc_x < -15:
        u_i[0, 0] = -15
    elif acc_x > 15:
        u_i[0, 0] = 15
    else:
        u_i[0, 0] = acc_x

    x_current = I(x0=x_current, p=u_i)["xf"]
    rx = x_current[0, 0]
    th = x_current[1, 0]

    img = copy.deepcopy(img_org)
    rx0 = width//2 + int(rx/csize)
    ry0 = height//2
    boxW = 40
    boxH = 20
    rx1 = width//2 + int(rx/csize) - boxW 
    rx2 = width//2 + int(rx/csize) + boxW
    ry1 = height//2 - boxH
    ry2 = height//2 + boxH
    cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (255, 0, 0), 2)

    cv2.line(img, (rx0, ry0), (rx0 - int(length*math.sin(th)), ry0 - int(length*math.cos(th))), (255, 0, 0), 2)
    cv2.imshow("sim", img)
    cv2.waitKey(5)
    
    # Windowの端で反射させる
    if rx1 < 0 or rx2 > width:
        x_current[2, 0] = -x_current[2, 0]
    
print("bye")
exit()
