import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# 1) DC Motor parametreleri
# ---------------------------
R = 1.0
L = 0.5
Kt = 0.01
Kb = 0.01
J = 0.01
B = 0.001
Vmax = 24.0

# ---------------------------
# 2) Üyelik Fonksiyonları
# ---------------------------
def triangular(x, a, b, c):
    x = np.asarray(x)
    mu = np.zeros_like(x, dtype=float)
    left = (a < x) & (x <= b)
    mu[left] = (x[left] - a) / (b - a + 1e-12)
    right = (b < x) & (x < c)
    mu[right] = (c - x[right]) / (c - b + 1e-12)
    mu[x == b] = 1.0
    return mu

# --- Güncellenmiş Üyelik Fonksiyonları ---
e_NB = (-400, -250, -150)
e_NS = (-200, -75, -10)
e_Z  = (-20, 0, 20)
e_PS = (10, 75, 200)
e_PB = (150, 250, 400)

de_N = (-200, -80, 0)
de_Z = (-30, 0, 30)
de_P = (0, 80, 200)

u_N = (-Vmax*1.5, -Vmax, 0)
u_Z = (-7, 0, 7)
u_P = (0, Vmax, Vmax*1.5)

output_mfs = {'N': u_N, 'Z': u_Z, 'P': u_P}

# Kural Tablosu (değişmedi)
rule_table = [
    ['N', 'N', 'N'],   # NB
    ['N', 'N', 'Z'],   # NS
    ['N', 'Z', 'P'],   # Z
    ['Z', 'P', 'P'],   # PS
    ['P', 'P', 'P']    # PB
]

# ---------------------------
# 3) Fuzzification
# ---------------------------
def fuzzify_e_de(e, de):
    mu_e = {
        'NB': triangular([e], *e_NB)[0],
        'NS': triangular([e], *e_NS)[0],
        'Z':  triangular([e], *e_Z)[0],
        'PS': triangular([e], *e_PS)[0],
        'PB': triangular([e], *e_PB)[0],
    }
    mu_de = {
        'N': triangular([de], *de_N)[0],
        'Z': triangular([de], *de_Z)[0],
        'P': triangular([de], *de_P)[0]
    }
    return mu_e, mu_de

# ---------------------------
# 4) Max–Product + LOM Defuzzification
# ---------------------------
def mamdani_defuzz(e, de, u_disc=np.linspace(-Vmax, Vmax, 1001)):
    mu_e, mu_de = fuzzify_e_de(e, de)
    aggregated = np.zeros_like(u_disc)
    e_labels = ['NB', 'NS', 'Z', 'PS', 'PB']
    de_labels = ['N', 'Z', 'P']

    for i_e, e_lab in enumerate(e_labels):
        for j_de, de_lab in enumerate(de_labels):
            fire = min(mu_e[e_lab], mu_de[de_lab])
            if fire <= 0: continue

            a, b, c = output_mfs[rule_table[i_e][j_de]]
            mu_out = triangular(u_disc, a, b, c)

            # *** MAX–PRODUCT ÇIKARIM ***
            aggregated = np.maximum(aggregated, mu_out * fire)

    # *** LOM (Largest of Maximum) BERRAKLAŞTIRMA ***
    return u_disc[np.argmax(aggregated)]

# ---------------------------
# 5) DC Motor Dinamiği
# ---------------------------
def motor_derivatives(x, u, TL=0.0):
    i, w = x
    di = (-R*i - Kb*w + u)/L
    dw = (-B*w + Kt*i - TL)/J
    return np.array([di, dw])

def rk4_step(x, u, dt, TL=0.0):
    k1 = motor_derivatives(x, u, TL)
    k2 = motor_derivatives(x + 0.5*dt*k1, u, TL)
    k3 = motor_derivatives(x + 0.5*dt*k2, u, TL)
    k4 = motor_derivatives(x + dt*k3, u, TL)
    return x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

# ---------------------------
# 6) Kapalı Çevrim Simülasyon
# ---------------------------
def simulate(ref_func, T=5.0, dt=0.001, x0=None):
    if x0 is None: x = np.array([0.0, 0.0])
    else: x = np.array(x0, dtype=float)

    t = np.arange(0, T+dt, dt)
    N = len(t)

    i_hist = np.zeros(N)
    w_hist = np.zeros(N)
    u_hist = np.zeros(N)

    prev_e = 0.0

    for k in range(N):
        ref = ref_func(t[k])
        e = ref - x[1]
        de = e - prev_e

        u = mamdani_defuzz(e, de)
        x = rk4_step(x, u, dt)

        i_hist[k], w_hist[k], u_hist[k] = x[0], x[1], u
        prev_e = e

    return t, w_hist, u_hist

# ---------------------------
# 7) Örnek Simülasyon
# ---------------------------
if __name__ == "__main__":
    ref_val = 100.0  # rad/s
    def ref(t):
        return ref_val * (t/0.2) if t < 0.2 else ref_val

    t, w, u = simulate(ref, T=5.0, dt=0.001)

    plt.figure(figsize=(10,5))
    plt.subplot(2,1,1)
    plt.plot(t, w, label='Hız (rad/s)')
    plt.axhline(ref_val, linestyle='--', label='Referans')
    plt.legend(); plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(t, u, label='Kontrol Sinyali (V)')
    plt.legend(); plt.grid(True)
    plt.tight_layout()
    plt.show()
