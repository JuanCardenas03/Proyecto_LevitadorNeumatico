import csv, sys, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'traza.csv')))
t = [float(x['t']) for x in r]; col = lambda k: [float(x[k]) for x in r]
S, T1, T2, B1, B2 = '#fcfcfb', '#0b0b0b', '#52514e', '#2a78d6', '#eb6834'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#c9c8c2', 'axes.labelcolor': T2,
                     'xtick.color': T2, 'ytick.color': T2})
fig, ax = plt.subplots(2, 1, figsize=(13, 6.5), sharex=True, gridspec_kw={'height_ratios': [2, 1]}, facecolor=S)
for a in ax:
    a.set_facecolor(S); a.grid(color='#e6e5e0', lw=0.8); a.spines[['top', 'right']].set_visible(False)
ax[0].plot(t, col('sp'), color=B2, lw=2, label='Setpoint')
ax[0].plot(t, col('altura_medida'), color=B1, lw=2, label='Altura medida por la PSoC')
ax[0].set_ylabel('cm'); ax[0].legend(loc='lower right', frameon=False, ncol=2, bbox_to_anchor=(1, 1.0))
ax[0].set_title('Firmware de la PSoC simulado en el PC (planta simulada, no identificada)', color=T1, loc='left')
ev = [(2.1, 'SP 15'), (9.05, 'SP 25'), (16.05, 'SP 8'), (22.1, 'Manual 52 %'), (26, 'Auto'), (32, 'EMG'),
      (36, 'ESP32 desconectada'), (41, 'EN:1'), (46, 'Sensor sin eco'), (53, 'Consola sp 20'), (60.1, 'Rampa'),
      (92, 'Manual'), (96, 'Fuente sim'), (98, 'sp 15 (sim)')]
for x, s in ev:
    ax[0].axvline(x, color='#c9c8c2', lw=0.8, ls='--')
    ax[0].text(x + 0.3, 33.5, s, rotation=90, va='top', fontsize=8, color=T2)
ax[1].plot(t, col('duty'), color=B1, lw=2)
ax[1].set_ylabel('PWM (%)'); ax[1].set_xlabel('tiempo (s)')
ax[0].set_ylim(-1, 35); ax[1].set_ylim(-3, 80); ax[1].set_xlim(-1, 105.5)
fig.tight_layout(); fig.savefig('simulacion_firmware.png', dpi=110, facecolor=S)
