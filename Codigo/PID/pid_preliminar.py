"""
===============================================================================
 DISENO DE UN PID PRELIMINAR
 Tarjeta [C-01] - Persona B (Control)  |  Proyecto Levitador Neumatico
===============================================================================

 QUE HACE:
   Calcula ganancias PID de partida a partir de los parametros identificados
   en la respuesta al escalon (zeta, wn), usando COLOCACION DE POLOS, y las
   verifica simulando el lazo cerrado con el modelo no lineal.

 IDEA DEL METODO:
   1) Se decide como se quiere que responda el lazo cerrado (velocidad wd y
      amortiguamiento zeta_d deseados, mas un polo real rapido p).
   2) Eso fija los polos deseados; igualando coeficientes de la ecuacion
      caracteristica se despejan Kp, Ki, Kd.
   3) Se simula con el modelo no lineal (con saturacion y zona muerta) para
      comprobar que el diseno funciona de verdad.

 LECCION CLAVE:
   El actuador satura (duty 0-100%) y el ventilador tiene retardo, asi que
   NO se puede hacer el lazo cerrado mucho mas rapido que la planta abierta.
   Pedir wd alto produce saturacion y oscilaciones. Hay que ser conservador.

 Estas ganancias son un PUNTO DE PARTIDA. Se afinaran sobre la planta real.
 Depende de modelo_levitador.py (misma carpeta).  Uso: python pid_preliminar.py
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from modelo_levitador import Parametros, integrar, linealizar

p = Parametros()

# --- Modelo de diseno (2do orden) alrededor de 45 cm -------------------------
wn, zeta, _, _ = linealizar(0.45, p)
v0_eq = p.v_eq * np.exp(0.45 / p.lam)
K_eff = p.lam * p.v0_max / v0_eq         # ganancia efectiva con comp. zona muerta
a = 2 * zeta * wn
b = wn**2
Kb = K_eff * b


def pole_placement(zeta_d, wd, pd):
    """Ganancias PID para colocar polos: par (zeta_d, wd) + polo real -pd."""
    Kd = (2*zeta_d*wd + pd - a) / Kb
    Kp = (wd**2 + 2*zeta_d*wd*pd - b) / Kb
    Ki = (wd**2 * pd) / Kb
    return Kp, Ki, Kd


# --- Simulador de lazo cerrado con compensacion de zona muerta ---------------
def sim(sp_func, Kp, Ki, Kd, Ts=0.020, T=12.0, x0=0.10, Tf=0.05, perturb=None):
    n = int(T / Ts)
    x, v, v0 = x0, 0.0, 0.0
    I, D, y_prev = 0.0, 0.0, x0
    af = Tf / (Tf + Ts)
    bf = Kd / (Tf + Ts)
    log = np.zeros((n, 5))
    for k in range(n):
        tk = k * Ts
        sp = sp_func(tk)
        e = sp - x
        D = af * D - bf * (x - y_prev)          # derivada sobre la medida
        y_prev = x
        u_unsat = Kp * e + I + D
        c = float(np.clip(u_unsat, 0.0, 1.0))
        if not (u_unsat > 1.0 and e > 0) and not (u_unsat < 0.0 and e < 0):
            I += Ki * e * Ts                    # anti-windup condicional
        I = float(np.clip(I, 0.0, 1.0))
        u = p.u_dz + c * (1 - p.u_dz) if c > 0 else 0.0   # comp. zona muerta
        if perturb is not None:
            v += perturb(tk) / p.m * Ts
        x, v, v0 = integrar(x, v, v0, u, p, Ts, sub=8)
        log[k] = [tk, x, c, e, sp]
    return log


def metricas(log, sp):
    tt, x = log[:, 0], log[:, 1]
    ov = max(0.0, x.max() - sp) / sp * 100
    ess = abs(x[-1] - sp) * 1000
    fuera = np.where(np.abs(x - sp) > 0.02 * sp)[0]
    ts = tt[fuera[-1]] if len(fuera) else 0.0
    return ov, ts, ess


if __name__ == "__main__":
    print(f"Modelo de diseno @45cm:  wn={wn:.2f}  zeta={zeta:.3f}  K_eff={K_eff:.2f}\n")

    # Diseno preliminar elegido: conservador (wd < wn) y bien amortiguado
    Kp, Ki, Kd = pole_placement(zeta_d=1.0, wd=2.5, pd=6)
    print(f">>> DISENO PRELIMINAR:  Kp={Kp:.2f}  Ki={Ki:.2f}  Kd={Kd:.2f}\n")

    print("Verificacion en todo el rango del tubo:")
    print(f"  {'objetivo':>10} {'OS %':>7} {'ts(2%)':>8} {'ess mm':>8}")
    for sp in [0.20, 0.30, 0.45, 0.60, 0.70]:
        ov, ts, ess = metricas(sim(lambda t: sp, Kp, Ki, Kd), sp)
        print(f"  {sp*100:8.0f}cm {ov:7.1f} {ts:8.2f} {ess:8.1f}")

    # --- Graficas ---
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))

    for sp, c in zip([0.25, 0.45, 0.65], ["#2ca02c", "#ff7f0e", "#9467bd"]):
        log = sim(lambda t: sp, Kp, Ki, Kd)
        ax[0,0].plot(log[:,0], log[:,1]*100, lw=2.5, color=c, label=f"{sp*100:.0f} cm")
        ax[0,0].axhline(sp*100, ls=":", color=c, alpha=0.5)
    ax[0,0].set_title(f"Diseno preliminar (Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f})")
    ax[0,0].set_xlabel("Tiempo [s]"); ax[0,0].set_ylabel("Altura [cm]")
    ax[0,0].grid(alpha=0.3); ax[0,0].legend()

    for kp, c in zip([0.2, 0.4, 0.8], ["#4575b4", "#000000", "#d73027"]):
        log = sim(lambda t: 0.45, kp, Ki, Kd)
        ax[0,1].plot(log[:,0], log[:,1]*100, lw=2.2, color=c, label=f"Kp={kp}")
    ax[0,1].axhline(45, ls=":", color="gray", alpha=0.6)
    ax[0,1].set_title("Efecto de Kp"); ax[0,1].set_xlabel("Tiempo [s]")
    ax[0,1].set_ylabel("Altura [cm]"); ax[0,1].grid(alpha=0.3); ax[0,1].legend()

    for ki, c in zip([0.3, 1.04, 2.5], ["#4575b4", "#000000", "#d73027"]):
        log = sim(lambda t: 0.45, Kp, ki, Kd)
        ax[1,0].plot(log[:,0], log[:,1]*100, lw=2.2, color=c, label=f"Ki={ki}")
    ax[1,0].axhline(45, ls=":", color="gray", alpha=0.6)
    ax[1,0].set_title("Efecto de Ki"); ax[1,0].set_xlabel("Tiempo [s]")
    ax[1,0].set_ylabel("Altura [cm]"); ax[1,0].grid(alpha=0.3); ax[1,0].legend()

    for kd, c in zip([0.0, 0.22, 0.5], ["#d73027", "#000000", "#4575b4"]):
        log = sim(lambda t: 0.45, Kp, Ki, kd)
        ax[1,1].plot(log[:,0], log[:,1]*100, lw=2.2, color=c, label=f"Kd={kd}")
    ax[1,1].axhline(45, ls=":", color="gray", alpha=0.6)
    ax[1,1].set_title("Efecto de Kd"); ax[1,1].set_xlabel("Tiempo [s]")
    ax[1,1].set_ylabel("Altura [cm]"); ax[1,1].grid(alpha=0.3); ax[1,1].legend()

    plt.tight_layout()
    plt.savefig("pid_preliminar.png", dpi=140)
    print("\n>> Grafica guardada: pid_preliminar.png")
