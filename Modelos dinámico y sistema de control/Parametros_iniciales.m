%% Parámetros físicos del drone

m = 2;%[kg]
I = [[0.01739,0,0];[0,0.01739,0];[0,0,0.0225]];
l = 0.2;
cd = 0.2;
%Para Axy simplemente multiplicar el número de rotores por el área de cada
%rotor, Ayz y Axz son iguales y se les dará un valor en función de otros
%modelos semejantes
Areavector = [0.293;0.022;0.022];

% cdy = 0.05;
% cdz = 0.05;

%% Tiempo de sampleo
ts = 0.01;

%% Estado inicial
Euler_0 = [0;0;0];
XYZ_0 = [0;0;5];
body_rate_0 = [0;0;0];

%% Entorno
g = [0;0;-3.66];
WindLevel = 1;
rho=0.0195;

%% Parámetro Viento
%Parámetros distribución Weibull
K = 1.2;
C = 3.85;