import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve

# Cargar los datos
try:
    # Si los valores están separados por espacios, usa el siguiente código:
    datos = np.loadtxt('Muestra30-100.txt', skiprows=2)
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
PWM = datos[:, 1]  # toma datos de 2da columna (convertir a tensión)
ADC_tacometro = datos[:, 2]  # tensión del tacómetro

# Acondicionamiento de señal
t = t / fs  # Tiempo
V_PWM = 12 * PWM / 100  # Transformo PWM a tensión de fuente
V_tacometro = ADC_tacometro * V_tacometro_max / (2 ** nro_bits)  # Lectura del ADC, en V

# Filtro FIR (media móvil)
n_fir = 32
coef = np.ones(n_fir) / n_fir  # Coeficientes del FIR de media móvil
V_tacometro_filt = convolve(V_tacometro, coef, mode='same')  # Aplica el filtro FIR

# Crear la figura y los subgráficos
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# Graficar las señales
ax1.plot(t, V_PWM, 'b', linewidth=2)
ax1.set_ylabel('V_PWM')
ax2.plot(t, V_tacometro_filt, 'r', linewidth=2)
ax2.set_ylabel('V_tacometro')
ax2.set_xlabel('t')

# Mostrar las gráficas
plt.show()
