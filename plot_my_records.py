import struct
import matplotlib.pyplot as plt
import pandas as pd
plt.rcParams['axes.grid'] = True

fileName = 'toto.bin'
# 'p2z2_results_4ms.bin'
# fileName =  'p2z2_results_600us.bin'
with open(fileName, 'rb') as f:
    raw_datas = f.read()

datas = [ {
    'v1_low' : a[0],
    'v2_low' : a[1],
    'vHigh'  : a[2],
    'iHigh'  : a[3],
    'i1_low' : a[4],
    'i2_low' : a[5],
    'iRef'   : a[6],
    'vRef'   : a[7],
    'tCalc'  : a[8]} for a in struct.iter_unpack("ffffffffQ", raw_datas)]

record = pd.DataFrame(datas);
time = [100e-6*k for k in range(len(record))];

fig, axs = plt.subplot_mosaic('''
AA
BB
''')

axs['A'].step(time, record.vRef, label='vRef', where='post');
axs['A'].step(time, record.v1_low, label='v1_low', where='post');
axs['A'].step(time, record.v2_low, label='v2_low', where='post');
axs['A'].step(time, record.vHigh, label='vHigh', where='post');
axs['A'].set_ylabel('voltage [V]');
axs['B'].step(time, record.iRef, label='iRef', where='post');
axs['B'].step(time, record.i1_low, label='i1_low', where='post');
axs['B'].step(time, record.i2_low, label='i2_low', where='post');
axs['B'].step(time, record.iHigh, label='iHigh', where='post');
axs['B'].set_ylabel('current [A]');

plt.legend();
# axs['C'].plot(time, record.tCalc*1e-3, label='calculation time');
# axs['C'].set_ylabel('computation tim [us]');
for ax in axs.values():
    #ax.set_xlim([0, 20e-3])
    ax.set_xlabel('time [s]')
    ax.legend()
fig.tight_layout()
plt.ion()
plt.show()
