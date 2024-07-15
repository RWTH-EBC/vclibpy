########################################################################################################################
# imports
########################################################################################################################

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

########################################################################################################################
# open csv
########################################################################################################################
file = open(r"D:\00_temp\screw_compressor\results5.csv")

Data = pd.read_csv(file)
#plt = Data_Optihorst.plot.hist(column="error_con in % (Procentual error of Condenser)")
#plt.show()


ax1 = Data.plot.scatter(x="T_out", y="err_h_3", c='DarkBlue', ylabel="err_h3")
ax1 = Data.plot.scatter(x="T_out", y="err_Q_amb", c='DarkRed', ylabel="err_Q_amb")
ax1 = Data.plot.scatter(x="T_out", y="err_m_flow", c='DarkGreen', ylabel="err_m_flow")
#ax1.set_xticks([0,100,200,300,400,500,600,700])
plt.show()

a = Data['T_out'].tolist()
fig, axs = plt.subplots()
axs.hist(a, bins=100)
plt.show()

