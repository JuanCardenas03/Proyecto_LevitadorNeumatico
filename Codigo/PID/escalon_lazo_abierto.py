"""
===============================================================================
 RESPUESTA AL ESCALON EN LAZO ABIERTO
 Tarjeta [C-01] - Persona B (Control)  |  Proyecto Levitador Neumatico
===============================================================================

 QUE ES:
   Aplicar un cambio brusco (escalon) en el duty del ventilador SIN controlador
   ni realimentacion, y observar como responde la pelota por si sola. Es el
   experimento base de identificacion de sistemas.

 PARA QUE SIRVE:
   1) Ahora (simulacion): verificar el modelo y extraer los parametros
      dinamicos (wn, zeta, ganancia) con los que se disena el PID.
   2) Despues (tarjeta [C-04], con la planta real): es EL metodo para
      identificar la planta fisica. Se aplica el mismo escalon real, se graba
      la altura con el sensor y se ajusta el modelo a esos datos.

 Este script depende de modelo_levitador.py (debe estar en la misma carpeta).
 Uso:  python escalon_lazo_abierto.py
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from modelo_levitador import (Parametros, integrar, duty_equilibrio,
                              altura_equilibrio, linealizar, RHO_AIRE)

p = Parametros()


# ---------------------------------------------------------------------------
def escalon(x_ini, u_ini, u_fin, t_step=1.0, T=12.0, Ts=0.002):
    """Aplica un escalon de duty en lazo abierto.
    Devuelve (tiempo, altura). El sistema arranca en equilibrio con u_ini."""
    x, v = x_ini, 0.0
    v0 = p.v0_max * ((u_ini - p.u_dz) / (1 - p.u_dz) if u_ini > p.u_dz else 0.0)
    t_arr, x_arr = [], []
    for k in range(int(T / Ts)):
        t = k * Ts
        u = u_ini if t < t_step else u_fin
        x, v, v0 = integrar(x, v, v0, u, p, Ts, sub=4)
        t_arr.append(t); x_arr.append(x)
    return np.array(t_arr), np.array(x_arr)


def identificar(t, x, t_step=1.0):
    """Extrae parametros de 2do orden SOLO desde la forma de la respuesta.
    Este es el procedimiento que se repetira con datos reales en [C-04]."""
    mask = t >= t_step
    tt, xx = t[mask] - t_step, x[mask]
    x_ini, x_fin = xx[0], xx[-1]
    delta = x_fin - x_ini

    # Sobreimpulso -> zeta
    pico = xx.max() if delta > 0 else xx.min()
    Mp = max((pico - x_fin) / delta if delta > 0 else (x_fin - pico) / (-delta), 1e-6)
    lnMp = np.log(Mp)
    zeta = -lnMp / np.sqrt(np.pi**2 + lnMp**2)

    # Periodo entre picos -> wn
    dx = np.diff(xx)
    picos = [i for i in range(1, len(dx)) if dx[i-1] > 0 and dx[i] <= 0]
    if len(picos) >= 2:
        Td = tt[picos[1]] - tt[picos[0]]
        wn = (2 * np.pi / Td) / np.sqrt(1 - zeta**2)
    else:
        Td, wn = np.nan, np.nan

    # Tiempo de establecimiento 2%
    fuera = np.where(np.abs(xx - x_fin) > 0.02 * abs(delta))[0]
    ts = tt[fuera[-1]] if len(fuera) else 0.0

    return dict(x_ini=x_ini*100, x_fin=x_fin*100, Mp=Mp*100,
                zeta=zeta, Td=Td, wn=wn, ts=ts)


# ===========================================================================
if __name__ == "__main__":
    # --- EXP 1: escalon en la mitad, con identificacion ---
    print("="*66)
    print(" EXP 1: escalon +5% desde 40 cm  (identificacion de parametros)")
    print("="*66)
    u0 = duty_equilibrio(0.40, p)
    t, x = escalon(0.40, u0, u0 + 0.05)
    r = identificar(t, x)
    wn_t, z_t, _, _ = linealizar(0.42, p)
    print(f"  Altura:  {r['x_ini']:.1f} -> {r['x_fin']:.1f} cm  "
          f"(estatica predice {altura_equilibrio(u0+0.05,p)*100:.1f} cm)")
    print(f"  Sobreimpulso {r['Mp']:.1f} %   |   ts(2%) {r['ts']:.2f} s   "
          f"|   periodo {r['Td']:.2f} s")
    print(f"  IDENTIFICADO:  zeta={r['zeta']:.3f}  wn={r['wn']:.2f} rad/s")
    print(f"  ANALITICO:     zeta={z_t:.3f}  wn={wn_t:.2f} rad/s")
    print(f"  (difieren porque la respuesta incluye la dinamica del ventilador)")

    # --- EXP 2: no linealidad ---
    print("\n" + "="*66)
    print(" EXP 2: mismo escalon +5% a 3 alturas -> la ganancia cambia")
    print("="*66)
    for x_op in [0.20, 0.45, 0.65]:
        uop = duty_equilibrio(x_op, p)
        _, xx = escalon(x_op, uop, uop + 0.05)
        salto = (xx[-1] - x_op) * 100
        print(f"  desde {x_op*100:>3.0f} cm -> salto {salto:5.1f} cm "
              f"(ganancia {salto/5:.2f} cm/%)")

    # --- Graficas ---
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))

    t, x = escalon(0.40, u0, u0 + 0.05); r = identificar(t, x)
    ax[0,0].plot(t, x*100, lw=2.5, color="#d62728")
    ax[0,0].axhline(r['x_fin'], ls="--", color="gray", label=f"final {r['x_fin']:.1f} cm")
    ax[0,0].axvline(1.0, ls=":", color="black", alpha=0.5)
    ax[0,0].set_title("(a) Como se lee la respuesta")
    ax[0,0].set_xlabel("Tiempo [s]"); ax[0,0].set_ylabel("Altura [cm]")
    ax[0,0].grid(alpha=0.3); ax[0,0].legend(fontsize=9)

    for x_op, c in zip([0.20,0.45,0.65], ["#2ca02c","#ff7f0e","#9467bd"]):
        uop = duty_equilibrio(x_op, p)
        t, x = escalon(x_op, uop, uop+0.05)
        ax[0,1].plot(t, x*100, lw=2.5, color=c, label=f"desde {x_op*100:.0f} cm")
    ax[0,1].axvline(1.0, ls=":", color="black", alpha=0.5)
    ax[0,1].set_title("(b) No linealidad: ganancia vs. altura")
    ax[0,1].set_xlabel("Tiempo [s]"); ax[0,1].set_ylabel("Altura [cm]")
    ax[0,1].grid(alpha=0.3); ax[0,1].legend(fontsize=9)

    uop = duty_equilibrio(0.45, p)
    t, x = escalon(0.45, uop, uop+0.06)
    ax[1,0].plot(t, x*100, lw=2.5, color="#1f77b4", label="subir (+6%)")
    t, x = escalon(0.45, uop, uop-0.06)
    ax[1,0].plot(t, x*100, lw=2.5, color="#e377c2", label="bajar (-6%)")
    ax[1,0].axvline(1.0, ls=":", color="black", alpha=0.5)
    ax[1,0].set_title("(c) Subir vs. bajar: asimetria")
    ax[1,0].set_xlabel("Tiempo [s]"); ax[1,0].set_ylabel("Altura [cm]")
    ax[1,0].grid(alpha=0.3); ax[1,0].legend(fontsize=9)

    uop = duty_equilibrio(0.40, p)
    for du, c in zip([0.02,0.05,0.10], ["#17becf","#bcbd22","#8c564b"]):
        t, x = escalon(0.40, uop, uop+du)
        ax[1,1].plot(t, x*100, lw=2.5, color=c, label=f"+{du*100:.0f}% duty")
    ax[1,1].axvline(1.0, ls=":", color="black", alpha=0.5)
    ax[1,1].set_title("(d) Escalones de distinto tamano")
    ax[1,1].set_xlabel("Tiempo [s]"); ax[1,1].set_ylabel("Altura [cm]")
    ax[1,1].grid(alpha=0.3); ax[1,1].legend(fontsize=9)

    plt.tight_layout()
    # Para guardar en tu carpeta, cambia el nombre por la ruta completa con r"..."
    plt.savefig("escalon_lazoabierto.png", dpi=140)
    print("\n>> Grafica guardada: escalon_lazoabierto.png")
