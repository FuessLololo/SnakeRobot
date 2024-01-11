# This script means to merge the .csv files in this directory into one .csv file
import pandas as pd
import os

folder_path = os.getcwd()
all_files = os.listdir(folder_path)

csv_files = [f for f in all_files if f.endswith('.csv')]

# sort the csv files by its name
csv_files.sort()

# Create a dataframe to store all the data
df_list = []

for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    df_list.append(df)
df = pd.concat(df_list)

df.to_csv(os.path.join(folder_path, 'speed.csv'), index=False)
