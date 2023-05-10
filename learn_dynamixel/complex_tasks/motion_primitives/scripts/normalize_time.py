import pandas as pd
import numpy as np
import glob
import sys

files = glob.glob(fr"{sys.argv[1]}*.csv")

for f in files:
    df = pd.read_csv(f)
    time = pd.DataFrame(np.linspace(0, 1, df.shape[0]))
    time.columns = ["time"]

    if df.columns[0] == "time":
        df = df.iloc[::, 1:]
        df_normalized_time = pd.concat([time, df], axis=1)
        df_normalized_time.to_csv(f"{f}", index=False)
    else:
        df_normalized_time = pd.concat([time, df], axis=1)
        df_normalized_time.to_csv(
            f"{f.split('.')[0]}_normalized_time.csv", index=False)
