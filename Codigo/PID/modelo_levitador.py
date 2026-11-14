"""
===============================================================================
 MODELO FISICO DEL LEVITADOR NEUMATICO
 Tarjeta [C-01] - Persona B (Control)
 Proyecto: Control PID con PSoC 5LP y comunicacion Modbus RTU / RS485
===============================================================================

 Este script implementa el modelo no lineal de la planta, su linealizacion
 analitica, y una simulacion del PID discreto con la MISMA estructura que se
 va a programar en el PSoC (periodo fijo, derivada sobre la medida, filtro
 derivativo, anti-windup por integracion condicional).

 IMPORTANTE: los parametros marcados como "DE AJUSTE" son estimaciones
 iniciales. Se reemplazan con los valores reales identificados en la tarjeta
 [C-04] una vez este armada la planta. Lo que importa AHORA es la ESTRUCTURA
 del modelo, no la exactitud de los numeros.

 Uso:  python3 modelo_levitador.py
===============================================================================
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from dataclasses import dataclass

# ----------------------------------------------------------------------------
# Constantes fisicas
# ----------------------------------------------------------------------------
G = 9.81           # gravedad [m/s^2]
RHO_AIRE = 1.20    # densidad del aire a ~25 C y 1000 m de altitud [kg/m^3]


# ============================================================================
# 1. PARAMETROS
# ============================================================================
@dataclass
class Parametros:
    """
    CONOCIDOS  -> se miden directamente con balanza y calibrador.
    DE AJUSTE  -> se identifican experimentalmente en [C-04].
    """
    # --- CONOCIDOS ---
    m: float = 2.7e-3        # masa de la pelota de ping-pong [kg]
    d: float = 0.040         # diametro de la pelota [m]
    L: float = 0.80          # longitud util del tubo [m]

    # --- DE AJUSTE ---
    Cd: float = 0.80         # coef. de arrastre EFECTIVO dentro del tubo.
                             # Una esfera libre tiene Cd~0.47, pero el
                             # confinamiento del tubo lo aumenta bastante.
    lam: float = 0.90        # longitud de decaimiento del chorro [m].
                             # ESTE ES EL PARAMETRO MAS IMPORTANTE: es el que
                             # hace que exista un equilibrio estable.
    v0_max: float = 20.0     # velocidad del aire en la base a duty 100% [m/s]
    tau_fan: float = 0.25    # constante de tiempo del ventilador [s]
    u_dz: float = 0.22       # zona muerta: duty minimo para que gire [0-1]

    @property
    def A(self):
        """Area frontal de la pelota [m^2]"""
        return np.pi * self.d**2 / 4.0

    @property
    def vol(self):
        """Volumen de la pelota [m^3]"""
        return (4.0/3.0) * np.pi * (self.d/2)**3

    @property
    def v_eq(self):
        """
        Velocidad de aire que equilibra exactamente el peso.
        RESULTADO CLAVE: es una CONSTANTE, no depende de la altura.
        De 1/2*rho*Cd*A*v^2 = m*g - empuje
        """
        peso_neto = self.m * G - RHO_AIRE * self.vol * G
        return np.sqrt(2.0 * peso_neto / (RHO_AIRE * self.Cd * self.A))


# ============================================================================
# 2. MODELO NO LINEAL
# ============================================================================
def u_efectivo(u, p):
    """Compensa la zona muerta del ventilador."""
    u = float(np.clip(u, 0.0, 1.0))
    return 0.0 if u < p.u_dz else (u - p.u_dz) / (1.0 - p.u_dz)


def v_aire(x, v0, p):
    """
    Perfil de velocidad del aire a lo largo del tubo.

    El chorro pierde velocidad con la altura por friccion viscosa contra
    las paredes y por ensanchamiento. Se modela con decaimiento exponencial.

    ESTE PERFIL ES EL CORAZON DEL MODELO: si la velocidad del aire fuera
    uniforme, NO habria equilibrio estable en ninguna altura (la pelota
    solo podria estar abajo o arriba del todo).
    """
    return v0 * np.exp(-max(x, 0.0) / p.lam)


def derivadas(x, v, v0, u, p):
    """Ecuaciones de estado del sistema completo (3 estados)."""
    va = v_aire(x, v0, p)
    v_rel = va - v

    # Arrastre. El producto v_rel*|v_rel| conserva el signo: si la pelota
    # sube mas rapido que el aire, el arrastre la frena.
    F_drag = 0.5 * RHO_AIRE * p.Cd * p.A * v_rel * abs(v_rel)
    F_flot = RHO_AIRE * p.vol * G       # empuje de Arquimedes (~1.5% del peso)

    a = (F_drag + F_flot) / p.m - G
    dv0 = (p.v0_max * u_efectivo(u, p) - v0) / p.tau_fan

    # Topes fisicos: rejilla abajo, tope arriba
    if x <= 0.0 and a < 0.0:
        v, a = max(v, 0.0), 0.0
    if x >= p.L and a > 0.0:
        v, a = min(v, 0.0), 0.0

    return v, a, dv0


def integrar(x, v, v0, u, p, dt, sub=10):
    """Integracion Euler con submuestreo."""
    h = dt / sub
    for _ in range(sub):
        dx, dv, dv0 = derivadas(x, v, v0, u, p)
        v += dv * h
        x = np.clip(x + dx * h, 0.0, p.L)
        v0 += dv0 * h
    return x, v, v0


# ============================================================================
# 3. ANALISIS ANALITICO
# ============================================================================
def altura_equilibrio(u, p):
    """
    Altura de equilibrio para un duty dado.
    En equilibrio: v_aire(x) = v_eq  ->  v0*exp(-x/lam) = v_eq
                   x_eq = lam * ln(v0 / v_eq)
    RELACION LOGARITMICA: esta es la no linealidad estatica principal.
    """
    v0 = p.v0_max * u_efectivo(u, p)
    return 0.0 if v0 <= p.v_eq else min(p.lam * np.log(v0 / p.v_eq), p.L)


def duty_equilibrio(x_eq, p):
    """Duty necesario para sostener la pelota a la altura x_eq."""
    ue = p.v_eq * np.exp(x_eq / p.lam) / p.v0_max
    return np.nan if ue > 1.0 else p.u_dz + ue * (1.0 - p.u_dz)


def linealizar(x_eq, p):
    """
    Linealizacion del subsistema de la pelota alrededor de x_eq.

    Derivacion:
      m*dv/dt = 1/2*rho*Cd*A*(va - v)|va - v| - m*g
      En equilibrio: 1/2*rho*Cd*A*v_eq^2 = m*g
      d(F)/d(va) = rho*Cd*A*v_eq = 2*m*g/v_eq
      d(F)/d(v)  = -2*m*g/v_eq
      d(va)/d(x) = -v_eq/lam

    Resultado:
      d2x/dt2 + (2g/v_eq)*dx/dt + (2g/lam)*x = (2g/v0_eq)*dv0

    Sistema de SEGUNDO ORDEN con:
      wn   = sqrt(2g/lam)        <- solo depende de lam
      zeta = g/(v_eq*wn)         <- solo depende de v_eq
      K_dc = lam/v0_eq           <- SI depende del punto de operacion
    """
    wn = np.sqrt(2.0 * G / p.lam)
    zeta = G / (p.v_eq * wn)
    v0_eq = p.v_eq * np.exp(x_eq / p.lam)
    K_dc = p.lam / v0_eq                    # [m / (m/s)]
    K_duty = K_dc * p.v0_max / (1 - p.u_dz)  # [m / duty]
    return wn, zeta, K_dc, K_duty


# ============================================================================
# 4. PID DISCRETO (misma estructura que ira en el firmware del PSoC)
# ============================================================================
class PID:
    def __init__(self, Kp, Ki, Kd, Ts, Tf=0.05, u_min=0.0, u_max=1.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.Ts, self.u_min, self.u_max = Ts, u_min, u_max
        self.a = Tf / (Tf + Ts)          # coef. del filtro derivativo
        self.b = Kd / (Tf + Ts)
        self.I = 0.0
        self.D = 0.0
        self.y_ant = None

    def calcular(self, sp, y):
        e = sp - y
        if self.y_ant is None:
            self.y_ant = y

        # DERIVADA SOBRE LA MEDIDA, no sobre el error.
        # Asi un cambio brusco de setpoint desde la HMI no produce un
        # "derivative kick" (pico enorme en la salida).
        self.D = self.a * self.D - self.b * (y - self.y_ant)
        self.y_ant = y

        u_sin_sat = self.Kp * e + self.I + self.D
        u = float(np.clip(u_sin_sat, self.u_min, self.u_max))

        # ANTI-WINDUP por integracion condicional:
        # no se integra si la salida esta saturada Y el error empuja
        # en la direccion que empeora la saturacion.
        saturado_arriba = u_sin_sat > self.u_max and e > 0
        saturado_abajo = u_sin_sat < self.u_min and e < 0
        if not (saturado_arriba or saturado_abajo):
            self.I += self.Ki * e * self.Ts
        self.I = float(np.clip(self.I, self.u_min, self.u_max))

        self.saturado = (u != u_sin_sat)
        return u, e


def simular_lazo_cerrado(p, perfil_sp, Kp, Ki, Kd, Ts=0.020, T=25.0,
                         x0=0.10, perturbacion=None):
    """perfil_sp(t) -> setpoint [m];  perturbacion(t) -> fuerza extra [N]"""
    n = int(T / Ts)
    x, v, v0 = x0, 0.0, 0.0
    pid = PID(Kp, Ki, Kd, Ts)
    log = np.zeros((n, 6))

    for k in range(n):
        t = k * Ts
        sp = perfil_sp(t)
        u, e = pid.calcular(sp, x)

        if perturbacion is not None:
            v += perturbacion(t) / p.m * Ts   # impulso de perturbacion

        x, v, v0 = integrar(x, v, v0, u, p, Ts)
        log[k] = [t, x, u, e, sp, pid.I]

    return log


def metricas(log, sp):
    t, x = log[:, 0], log[:, 1]
    sobreimpulso = max(0.0, x.max() - sp) / sp * 100.0
    ess = abs(x[-1] - sp) * 1000.0
    fuera = np.where(np.abs(x - sp) > 0.02 * sp)[0]
    ts = t[fuera[-1]] if len(fuera) else 0.0
    return sobreimpulso, ts, ess


# ============================================================================
# 5. EJECUCION
# ============================================================================
if __name__ == "__main__":
    p = Parametros()

    print("=" * 68)
    print(" PARAMETROS DERIVADOS")
    print("=" * 68)
    print(f"  Area frontal        : {p.A*1e4:.2f} cm2")
    print(f"  Peso                : {p.m*G*1000:.2f} mN")
    print(f"  Empuje (Arquimedes) : {RHO_AIRE*p.vol*G*1000:.3f} mN "
          f"({100*RHO_AIRE*p.vol/p.m:.1f}% del peso -> despreciable)")
    print(f"  v_eq (constante)    : {p.v_eq:.2f} m/s")

    wn, zeta, K_dc, K_duty = linealizar(0.45, p)
    print(f"\n  Modelo lineal @45cm:")
    print(f"    wn    = {wn:.2f} rad/s  ({wn/(2*np.pi):.2f} Hz)")
    print(f"    zeta  = {zeta:.3f}   -> MUY SUBAMORTIGUADO")
    print(f"    K     = {K_duty:.2f} m/duty")
    print(f"    Periodo de oscilacion natural = "
          f"{2*np.pi/(wn*np.sqrt(1-zeta**2)):.2f} s")

    print("\n" + "=" * 68)
    print(" CURVA ESTATICA Y VARIACION DE GANANCIA")
    print("=" * 68)
    print(f"  {'x_eq':>8} {'duty':>8} {'K [m/duty]':>12}")
    for xe in [0.15, 0.30, 0.45, 0.60, 0.70]:
        _, _, _, Kd_op = linealizar(xe, p)
        print(f"  {xe*100:6.0f}cm {duty_equilibrio(xe,p):8.3f} {Kd_op:12.2f}")
    r = linealizar(0.15, p)[3] / linealizar(0.70, p)[3]
    print(f"\n  >> La ganancia varia {r:.2f}x entre 15 y 70 cm.")
    print( "     Un PID sintonizado abajo sera lento arriba, y uno")
    print( "     sintonizado arriba sera oscilatorio abajo.")

    # ---- Sintonizacion propuesta -------------------------------------------
    Kp, Ki, Kd = 0.5, 0.7, 0.2
    print("\n" + "=" * 68)
    print(f" DESEMPENO PID  (Kp={Kp}, Ki={Ki}, Kd={Kd}, Ts=20ms)")
    print("=" * 68)
    print(f"  {'setpoint':>10} {'OS [%]':>9} {'ts 2% [s]':>11} {'ess [mm]':>10}")
    for sp in [0.20, 0.30, 0.45, 0.60, 0.70]:
        lg = simular_lazo_cerrado(p, lambda t, s=sp: s, Kp, Ki, Kd)
        ov, ts, ess = metricas(lg, sp)
        print(f"  {sp*100:8.0f}cm {ov:9.1f} {ts:11.2f} {ess:10.1f}")

    # ========================================================================
    # GRAFICAS
    # ========================================================================
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))

    # (a) Curva estatica
    us = np.linspace(0, 1, 400)
    xs = [altura_equilibrio(u, p) * 100 for u in us]
    ax[0, 0].plot(us * 100, xs, lw=2, color="#1f77b4")
    ax[0, 0].axvspan(0, p.u_dz * 100, alpha=0.15, color="red",
                     label="zona muerta")
    ax[0, 0].set_xlabel("Duty PWM [%]")
    ax[0, 0].set_ylabel("Altura de equilibrio [cm]")
    ax[0, 0].set_title("(a) Curva estatica: relacion logaritmica")
    ax[0, 0].grid(alpha=0.3); ax[0, 0].legend()

    # (b) Escalon en lazo abierto
    u0 = duty_equilibrio(0.40, p)
    x, v, v0 = 0.40, 0.0, p.v_eq * np.exp(0.40 / p.lam)
    Ts, T = 0.005, 12.0
    tt, xx = [], []
    for k in range(int(T / Ts)):
        t = k * Ts
        u = u0 if t < 1.0 else u0 + 0.05
        x, v, v0 = integrar(x, v, v0, u, p, Ts, sub=2)
        tt.append(t); xx.append(x * 100)
    ax[0, 1].plot(tt, xx, lw=2, color="#d62728")
    ax[0, 1].axhline(altura_equilibrio(u0 + 0.05, p) * 100, ls="--",
                     color="gray", label="equilibrio teorico")
    ax[0, 1].set_xlabel("Tiempo [s]"); ax[0, 1].set_ylabel("Altura [cm]")
    ax[0, 1].set_title("(b) Lazo abierto: escalon de +5% de duty")
    ax[0, 1].grid(alpha=0.3); ax[0, 1].legend()

    # (c) Lazo cerrado, varios setpoints
    for sp, c in zip([0.25, 0.45, 0.65], ["#2ca02c", "#ff7f0e", "#9467bd"]):
        lg = simular_lazo_cerrado(p, lambda t, s=sp: s, Kp, Ki, Kd, T=20.0)
        ax[1, 0].plot(lg[:, 0], lg[:, 1] * 100, lw=2, color=c,
                      label=f"sp = {sp*100:.0f} cm")
        ax[1, 0].axhline(sp * 100, ls=":", color=c, alpha=0.5)
    ax[1, 0].set_xlabel("Tiempo [s]"); ax[1, 0].set_ylabel("Altura [cm]")
    ax[1, 0].set_title(f"(c) Lazo cerrado PID ({Kp}, {Ki}, {Kd})")
    ax[1, 0].grid(alpha=0.3); ax[1, 0].legend()

    # (d) Perfil de setpoint + perturbacion
    def perfil(t):
        if t < 5:    return 0.25
        elif t < 15: return 0.55
        elif t < 25: return 0.35
        else:        return 0.60

    def perturb(t):
        # Impulso equivalente a un golpecito: dv = F/m*dt ~ 0.3 m/s
        return -0.004 if 19.9 < t < 20.1 else 0.0

    lg = simular_lazo_cerrado(p, perfil, Kp, Ki, Kd, T=35.0,
                              perturbacion=perturb)
    ax[1, 1].plot(lg[:, 0], lg[:, 4] * 100, ls="--", color="gray",
                  label="setpoint")
    ax[1, 1].plot(lg[:, 0], lg[:, 1] * 100, lw=2, color="#1f77b4",
                  label="altura")
    ax2 = ax[1, 1].twinx()
    ax2.plot(lg[:, 0], lg[:, 2] * 100, lw=1, color="#8c564b", alpha=0.5)
    ax2.set_ylabel("Duty [%]", color="#8c564b")
    ax[1, 1].axvline(20, color="red", ls=":", alpha=0.7)
    ax[1, 1].annotate("perturbacion", xy=(20, 65), color="red", fontsize=9)
    ax[1, 1].set_xlabel("Tiempo [s]"); ax[1, 1].set_ylabel("Altura [cm]")
    ax[1, 1].set_title("(d) Cambios de setpoint y rechazo de perturbacion")
    ax[1, 1].grid(alpha=0.3); ax[1, 1].legend(loc="lower right")

    plt.tight_layout()
    plt.savefig("simulacion_levitador.png", dpi=140)
    print("\n>> Grafica guardada: simulacion_levitador.png")
