clear all;
close all;
clc;

% importa datos de ADC
datos = importdata('Muestra2.txt',' ',3);

V_tacometro_max = 5.8 %5,8V determinado por ensayo de pwm=100%
R1 = 10000 %ohm
R2 = 10000 %ohm
V_tacometro_divisor = V_tacometro_max * R2/(R1+R2)

nro_bits = 10
fs = 1000 %frecuencia de muestreo, en Hz


t=datos.data(:,1);   %nro. de muestra (base de tiempo)
PWM=datos.data(:,2); %toma datos de 2da columna (convertir a tensión)
ADC_tacometro=datos.data(:,3); %tensión del tacómetro

% -------------- no entiendo esto
t0=t(1);
t=t-t0; %elimina 1er valor? no entiendo
% --------------

%Acondicionamiento de señal
t=t/fs
V_PWM = 12*PWM/100 %transformo PWM a tensión de fuente
V_tacometro = ADC_tacometro * V_tacometro_max/(2^nro_bits) %lectura del ADC, en V

%Filtro FIR, para limpiar la señal de salida por ruido de HF
n_fir = 32;
coef = ones(1, n_fir) / n_fir; % Coeficientes del FIR de media móvil (para convolucion
V_tacometro = conv(V_tacometro, coef, 'same'); % Aplica el filtro FIR

figure();
ax1=subplot(2,1,2),plot(t,V_tacometro,'r','lineWidth',2);
xlabel('t');ylabel('V_tacometro');
ax2=subplot(2,1,1),plot(t,V_PWM,'b','lineWidth',2);
xlabel('t');ylabel('V_PWM');
linkaxes([ax1,ax2],'x');