########################################################################################################################
# imports
########################################################################################################################

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

########################################################################################################################
# open csv
########################################################################################################################
file = open(r"D:\00_temp\screw_compressor\results8.csv")

Data = pd.read_csv(file)
#plt = Data_Optihorst.plot.hist(column="error_con in % (Procentual error of Condenser)")
#plt.show()


ax1 = Data.plot.scatter(x="x"[0] , y="err_h_3", c='DarkBlue', ylabel="err_h3")
ax1.set_xticks([320,360, 400,440,480])
#ax1 = Data.plot.scatter(x="T_out", y="err_Q_amb", c='DarkRed', ylabel="err_Q_amb")
ax1 = Data.plot.scatter(x="x"[0], y="err_m_flow", c='DarkGreen', ylabel="err_m_flow")
ax1.set_xticks([280,360, 400,440,480])
plt.show()

a = Data['T_out'].tolist()
fig, axs = plt.subplots()
axs.hist(a, bins=100)
plt.show()

