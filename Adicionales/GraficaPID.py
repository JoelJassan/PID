import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve

# Cargar los datos
try:
    datos = np.loadtxt('MuestraPID.txt', delimiter=',',skiprows=2)
    print("Datos cargados correctamente:")
except Exception as e:
    print(f"Error al cargar los datos: {e}")

# Definir las constantes
V_tacometro_max = 5.8  # 5,8V determinado por ensayo de pwm=100%
R1 = 10000  # ohm
R2 = 10000  # ohm
nro_bits = 10
fs = 1000  # frecuencia de muestreo, en Hz

# Variables del archivo de datos
t = datos[:, 0]  # nro. de muestra (base de tiempo)
PWM = datos[:, 1]  # toma datos de 2da columna (convertir)
set_point = datos[:, 2]  # toma datos de 3ra columna (en V)
V_tacometro = datos[:, 3]  # tensión del tacómetro

# Acondicionamiento de señal
t = t / fs  # Tiempo
V_PWM = 12 * PWM / 100  # Transformo PWM a tensión de fuente

# Filtro FIR (media móvil) eso no se usa porque esta integrado en el micro
n_fir = 32
coef = np.ones(n_fir) / n_fir  # Coeficientes del FIR de media móvil
V_tacometro_filt = convolve(V_tacometro, coef, mode='same')  # Aplica el filtro FIR

# Crear la figura y los subgráficos
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Graficar las señales
ax1.plot(t, PWM, 'b', linewidth=2)
ax1.set_ylabel('V_PWM')
ax2.plot(t, V_tacometro, 'r', linewidth=2)
ax2.plot(t, set_point, 'y', linewidth=1)
ax2.set_ylabel('V_tacometro y set_point')
ax2.set_xlabel('t')

# Mostrar las gráficas
plt.show()
