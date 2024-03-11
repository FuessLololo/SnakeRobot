import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

dir = os.path.dirname(__file__) + "/speed_sorted.csv"
tmp = pd.read_csv(dir)
x_data = tmp['Configuration']
x_data = map(lambda x: x[1:], x_data)
x_data = np.array(list(x_data))
print(len(x_data))
y_data = np.array(list(tmp['Speed']))
print(len(y_data))

plt.plot(x_data, y_data)
plt.title('Friction Configurations and Their Speeds')
plt.xlabel('Friction Configuration')
plt.ylabel('Speed')
plt.ylim(0, 2.25)
plt.ylabel('Speed (m/s)')

# hide the x-axis
ax = plt.gca()
ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False) 

plt.savefig(f'speed_sorted.png')

plt.show()