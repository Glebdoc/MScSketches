import numpy as np
import pandas as pd
# read a velocity field from CFD results 

df = pd.read_csv("SideViewVelocity.tsv", sep="\t")

# Display the first few rows
print(df.head())