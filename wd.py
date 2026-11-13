from rov_sim import *
s = RovSimulator(); s.arm()
for _ in range(100):
    s.set_manual_command(1, 0, 0, 0); s.step(0.02)
print("key held 2 s    ->", round(s.thrust[0], 2))
for _ in range(40): s.step(0.02)
print("0.8 s of silence ->", round(s.thrust[0], 2))
for _ in range(20): s.step(0.02)
print("1.2 s of silence ->", s.thrust[0])