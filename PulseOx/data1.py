# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.pylab as pylab
#import matplotlib.mlab as mlab
# import matplotlib.cm as cm
# import scipy.io.wavfile as wav
from scipy.optimize import curve_fit
import pandas as pd
import collections

plt.close('all')

df = pd.read_csv('pulsebigcover1.csv'); 

time = df.iloc[-1]['415']
df.drop(df.index[-(600)], inplace=True)

df = df.loc[0:99]

t = np.linspace(0,  time/1000000 , len(df['415']))

for i,col in enumerate(df.columns[1:-4]):
    plt.plot(df['time'], df[col]- np.average(df[col]) + i*20 , label = col)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('Centered around average ofset by 20 from eachother')
plt.legend()
plt.show()
    

print('Heartrate = ', np.average(df.loc[df['Confidence'] > 50 & (df['Heartrate'] > 0)]['Heartrate']))
print('Oxygen = ', np.average(df.loc[(df['Confidence'] > 50) & (df['Oxygen'] > 0.1)]['Oxygen']))
print('R = ',np.average(df.loc[(df['Confidence'] > 50) & (df['R'] > 0.1)]['R']))
print('Confidence = ', np.average(df.loc[df['Confidence'] > 50]['Confidence']))

differences = []
x = 0
for i in df['time']:
    differences.append(i-x)
    x = i
    
print(max(differences))
print(min(differences))
print(np.average(differences))

print(collections.Counter(differences))
