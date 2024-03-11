import pandas as pd
import matplotlib.pyplot as plt
import os

x_data = []
y_data = []

for affix in range(1, 17):
    if affix < 10:
        affix_ = '0' + str(affix)
    else:
        affix_ = str(affix)
    csv_file = 'speeds' + str(affix_) + '.csv'
    csv_file_path = os.path.join(os.path.dirname(__file__), csv_file)
    data = pd.read_csv(csv_file_path)

    x_data_cur = data['Configuration']
    x_data_cur = map(lambda x: x[1:], x_data_cur)
    y_data_cur = data['Speed']

    x_data += list(x_data_cur)
    y_data += list(y_data_cur)

x_data_df = pd.DataFrame(x_data)
print(x_data_df.shape) # check the shape of the dataframe

plt.plot(x_data, y_data)

plt.title('Friction Configurations and Their Speeds')
plt.xlabel('Friction Configuration')
plt.ylabel('Speed')
plt.ylim(0, 2.25)
plt.ylabel('Speed (m/s)')

plt.xticks([])

plt.savefig(f'pattern_speed13_16.png')

plt.show()