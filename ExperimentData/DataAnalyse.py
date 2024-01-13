import pandas as pd

def get_idx(df, col, val):
    """Return the index of the row where the value in the column is equal to val"""
    return df[df[col] == val].index[0]

# Read the data
df = pd.read_csv("speed.csv")
# Compute the averge speed and the standard deviation
avg_speed = df["Speed"].mean()
std_speed = df["Speed"].std()
# Find the largest speed and the row number
max_speed = df["Speed"].max()
max_speed_idx = get_idx(df, "Speed", max_speed)
# Find the small speeds and their row numbers
small_speeds_threshold = avg_speed - std_speed * 1.5
small_speeds = df[df["Speed"] < small_speeds_threshold]
small_speeds_idxs = small_speeds.index

# find the largest yaw rate and its row number
max_yaw_rate = df["Yaw Rate"].abs().max()
if max_yaw_rate != df["Yaw Rate"].max():
    max_yaw_rate_idx = get_idx(df, "Yaw Rate", -max_yaw_rate)
    max_yaw_rate = -max_yaw_rate
else:
    max_yaw_rate_idx = get_idx(df, "Yaw Rate", max_yaw_rate)

print(f"Average speed: {avg_speed}")
print(f"max speed: {max_speed}, its configuration is {df.iloc[max_speed_idx, 0]}")
print(f"small speeds: {small_speeds}")
print(f"max yaw rate: {max_yaw_rate}, its configuration is {df.iloc[max_yaw_rate_idx, 0]}")