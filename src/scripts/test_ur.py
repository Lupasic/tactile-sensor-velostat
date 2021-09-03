import urx
from time import perf_counter
from numpy import array, frexp, sin, cos, pi 
address="172.31.1.25"
rob = urx.Robot(address, use_rt=False)

print(rob)
rob.speedx("speedl",[0,0,0.0,0,0,0], 0.5, 5)

k = 0.5
A = 0.1
freq = 0.5
omega = 2*pi*freq
A0 = 0.5

try: 
    t0 = perf_counter()
    while True: 
        t = perf_counter() -t0
        state = rob.getl()        

        z_d = A0 + A*sin(omega*t)
        dz_d = A*omega*cos(omega*t)
        x ,y ,z = array(state[:3]) 
        u_z = dz_d + k*(z_d - z)
        rob.speedx("speedl",[0,0,u_z,0,0,0], 0.5, 5)
        
        print(f'{z.round(3)} {z_d.round(3)} {z_d.round(3) - z.round(3)}', end = '\r', flush = True)

except KeyboardInterrupt:
    rob.speedx("speedl",[0,0,0,0,0,0], 0.5, 1)
    print('Robot is stopped')
# except:
#     rob.speedx("speedl",[0,0,0,0,0,0], 0.5, 1)
