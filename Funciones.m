%sen^-1 = (20cm/30cm) resultado en radianes * 57,2958 para obtener grados
alpha = asin(2/3)*57.2958


%Constantes y funciones de transferencia
close all; clear all; clc

s=tf('s');
%FT del motor
W_V = 104.72/(s+2.5)

%Constante helice
h = 8.42e-3

%FT de Planta
T_F = 29.33/(s^2 + 1.778*s + 13.47)

%Constante conversor grados a radianes
gr = 57.2958

%Funcion V y grados
V_T = 0.02

%Sistema
FTLA = W_V*h*T_F*gr*V_T

FTLC = minreal(FTLA/(1+FTLA))

%Estabilidad Routh Hurwitz
a0 = 1;
a2 = 17.91;
a1 = 4.278;
a3 = 33.68;

b1 = (a1*a2 - a0*a3)/a1
c1 = (b1*a3 - a1*0)/b1

%Funcion de transferencia planta
It = 10.23e-3;
wn = 3.67;
cita = 4/(4.5*wn)
k = .3/(wn^2*It)

FT = (k*wn^2)/(s^2+2*cita*wn*s+wn^2)

%Errores en regimen permanente
%Kp error de posicion
%Kv error de velocidad
%Ka error de aceleracion

Kp=FTLA;
Kv=simplify(s*FTLA); 
Ka=simplify(s^2*FTLA);
s = 0;

eval(Kp)
eval(Kv)
eval(Ka)

% Error para las distintas entradas
ess_e=eval(1/(1+Kp)) 

%Comparacion de la respuesta del sistema ante un escalon unitario
t = 0:0.01:30; % Vector de tiempo
u = ones(size(t)); % Señal escalón unitario

[y,t] = step(FTLC,t);
plot(t,u)
hold on;
plot(t,y)
grid on;

%Diseño del compensador PID
d = [1 1.778 13.47];
Td=1/d(2); 
Ti=1/(d(3)*Td); 
Kp=1;

PID = tf(Kp*[Ti*Td Ti 1],[Ti 0])
rlocus(PID*FTLA);
step(FTLC,feedback(0.0937*PID*FTLA,1)
legend('Sist. Lazo cerrado sin compensar', 'Sist. lazo cerrado compensado')