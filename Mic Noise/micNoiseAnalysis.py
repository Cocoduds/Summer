# -*- coding: utf-8 -*-
"""
Created on Mon Jul 15 15:10:53 2024

@author: dudle
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.fft import fft, ifft, fftfreq
import pandas as pd
import collections
import glob


# def AvgNoiseSpec(file1, file2):
#     df = pd.read_csv(file1)
#     freq = df.iloc[-1]['raw data']
#     df.drop(df.index[-1], inplace=True)
#     data1 = df['raw data'].to_numpy()
#     # Number of samplepoints
#     N = len(data1)
#     # sample spacing
#     T = 1/freq
#     yf1 = fft(data1)
#     xf1 = fftfreq(N, T)[:N//2]
    
#     df = pd.read_csv(file1)
#     freq = df.iloc[-1]['raw data']
#     df.drop(df.index[-1], inplace=True)
#     data2 = df['raw data'].to_numpy()
#     # Number of samplepoints
#     N = len(data2)
#     # sample spacing
#     T = 1/freq
#     yf2 = fft(data2)
#     xf2 = fftfreq(N, T)[:N//2]
    
#     xf = (xf1 + xf2)/2
#     yf = (yf1 + yf2)/2
    
#     return xf,yf,N


def AvgNoiseSpec(folder):
    files = glob.glob(folder+'*.csv')
    xfs = []
    yfs =[]
    for file in files:
        df = pd.read_csv(file)
        freq = df.iloc[-1]['raw data']
        df.drop(df.index[-1], inplace=True)
        data1 = df['raw data'].to_numpy()
        # Number of samplepoints
        N = len(data1)
        # sample spacing
        T = 1/freq
        yfs.append(fft(data1))
        xfs.append(fftfreq(N, T)[:N//2])
        
    xf = sum(xfs)/len(xfs)
    yf = sum(yfs)/len(yfs)
    return xf,yf,N
    
    


plt.figure(1)
files = glob.glob('*.csv')

for file in files:
    df1 = pd.read_csv(file)
    plt.plot(np.linspace(0,5,len(df1)), df1, label = file)
    
    print(file, ': ', np.mean(df1))
    
plt.legend()
plt.xlabel("Time (minutes)")
plt.ylabel("RMS noise")
plt.show() 



#%%

plt.figure(2)

xf, yf, N = AvgNoiseSpec("2 Layer board/usb")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = '2 Layers')

xf, yf, N = AvgNoiseSpec("bat/usb")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = '4 layers')

plt.ylim(0, 10)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (V)')
plt.title('USB')
plt.legend()
plt.show()



plt.figure(3)

xf, yf, N = AvgNoiseSpec("2 Layer board/battery")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = '2 Layers')

xf, yf, N = AvgNoiseSpec("bat/battery")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = '4 layers')

plt.ylim(0, 10)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (V)')
plt.title('Li Ion Battery')
plt.legend()
plt.show()



#%%
plt.figure(4)

xf, yf, N = AvgNoiseSpec("M4 usb bat/")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'M4')

xf, yf, N = AvgNoiseSpec("M0 usb bat/")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'M0')

plt.ylim(0, 2)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (V)')
plt.title('2*AA - usb')
plt.legend()
plt.show()

plt.figure(5)

xf, yf, N = AvgNoiseSpec("M4 usb reg/")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'M4')

xf, yf, N = AvgNoiseSpec("M0 usb reg/")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'M0')

plt.ylim(0, 2)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (V)')
plt.title('regulator - usb')
plt.legend()
plt.show()

#%%

plt.figure(6)
df = pd.read_csv('tft on.csv')
plt.plot(np.linspace(0,5,len(df)), df,label = 'battery tft on')
df = pd.read_csv('tft off.csv')
plt.plot(np.linspace(0,5,len(df)), df,label = 'battery tft off')


df = pd.read_csv('tft on2.csv')
plt.plot(np.linspace(0,5,len(df)), df, label = 'm4 tft on')
df = pd.read_csv('tft off2.csv')
plt.plot(np.linspace(0,5,len(df)), df, label = 'm4 tft off')
plt.legend()
plt.show()

#%%
plt.figure(7)
xf, yf, N = AvgNoiseSpec("tft on")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT on')

xf, yf, N = AvgNoiseSpec("tft off")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT off')

plt.ylim(0, 10)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (adc)')
plt.title('Mic on battery')
plt.legend()
plt.show()


plt.figure(8)
xf, yf, N = AvgNoiseSpec("tft on2")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT on')

xf, yf, N = AvgNoiseSpec("tft off2")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT off')

plt.ylim(0, 10)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (adc)')
plt.title('Mic on m4')
plt.legend()
plt.show()

#%%

plt.figure(9)
xf, yf, N = AvgNoiseSpec("0807/2layeron")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT on')

xf, yf, N = AvgNoiseSpec("0807/2layeroff")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT off')

plt.ylim(0, 10)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (adc)')
plt.title('2 Layer - Mic on battery')
plt.legend()
plt.show()


plt.figure(10)
xf, yf, N = AvgNoiseSpec("0807/4layeron2")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT on')

xf, yf, N = AvgNoiseSpec("0807/4layeroff")
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]), label = 'TFT off')

plt.ylim(0, 10)
plt.xlim(0,600)
plt.xlabel('fq (Hz)')
plt.ylabel('Amplitude (adc)')
plt.title('4 Layer - Mic on battery')
plt.legend()
plt.show()

#%%

