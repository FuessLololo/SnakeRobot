import numpy as np

flag = []
for _ in range(16):
    rand_float = np.random.rand()
    if rand_float <= 0.1:
        flag.append(1)
    else:
        flag.append(0)

print(flag)
