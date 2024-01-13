import pandas as pd
import matplotlib.pyplot as plt

csv_file_path = 'speeds01.csv'  
data = pd.read_csv(csv_file_path)


x_data = data['Configuration']  
y_data = data['Speed']  


plt.plot(x_data, y_data)


plt.title('Friction Configurations and Their Speeds')
plt.xlabel('Friction Configuration')
plt.ylabel('Speed')

plt.xticks([])

plt.savefig('pattern_speed01.png')

plt.show()