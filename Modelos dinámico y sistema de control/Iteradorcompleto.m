%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Parámetros iniciales  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = 2;
I = [[0.01739,0,0];[0,0.01739,0];[0,0,0.0225]];
l = 0.2;
cd = 0.2;
Areavector = [0.293;0.022;0.022];ts = 0.01;
Euler_0 = [0;0;0];
XYZ_0 = [0;0;0];
body_rate_0 = [0;0;0];
g = [0;0;-3.66];
densidadbaterias = 161; %Wh/kg
rho=0.0195;
K = 1.2;
C = 3.85;











%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Cargar matriz ruta    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('Waypoints_matrix.mat')
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   XYZsignal_Waypoints   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
dX = []      
dY = []
dZ = []

for i = 1:length(WayPts)-2
    dX(i) = (WayPts(i+2,1)-WayPts(i+1,1))/(WayPts(i+2,4)-WayPts(i+1,4))
    dY(i) = (WayPts(i+2,2)-WayPts(i+1,2))/(WayPts(i+2,4)-WayPts(i+1,4))
    dZ(i) = (WayPts(i+2,3)-WayPts(i+1,3))/(WayPts(i+2,4)-WayPts(i+1,4))
end

Tfinal = WayPts(length(WayPts),4) + 10;
TOFtime = 1;
tss = 0.1;
t = 0:tss:Tfinal;

% Superposición de señales
x_sum = 0*t;
z_sum = 0*t;
y_sum = 0*t;
for k = 1:length(WayPts)
    if k == 1 % time: 0-1 sec, 1st Waypts
        section = [0, WayPts(k,4)];
        start_section = section(1);
        end_section = section(2);
        
        for i = 1:length(z_sum)
            if t(i) >= start_section && t(i) <= end_section
                z_sum(i) = 0;
                y_sum(i) = 0; 
                x_sum(i) = 0;
            end
        end
        
    elseif k == 2 %tiempo: 1-2 sec, 2nd Waypts, proceso de despegue
        section = [WayPts(k-1,4),WayPts(k,4)]
        start_section = section(1);
        end_section = section(2);
        
        for i = 1:length(z_sum)
            if t(i) > start_section && t(i) <= end_section
                z_sum(i) = WayPts(k,3);
                y_sum(i) = 0;
                x_sum(i) = 0;
            end
        end
        
    else % time: 2 - last Waypts sec, free fly mode
        section = [WayPts(k-1,4),WayPts(k,4)]
        start_section = section(1);
        end_section = section(2);
        
        for i = 1:length(z_sum)            
            if t(i) > start_section && t(i) <= end_section
                z_sum(i) = z_sum(i-1) + tss*dZ(k-2);
                y_sum(i) = y_sum(i-1) + tss*dY(k-2);
                x_sum(i) = x_sum(i-1) + tss*dX(k-2);
            end
        end
        
    end
end

start_section = WayPts(length(WayPts),4)
end_section = Tfinal
for i = 1:length(z_sum)
    if t(i) > start_section && t(i) <= end_section
        z_sum(i) = z_sum(i-1)
        y_sum(i) = y_sum(i-1)
        x_sum(i) = x_sum(i-1)
    end
end

% Proceso de creación de datos para cmd
sigZ = [t;z_sum]
Zcmd = timeseries(sigZ(2:end,:),sigZ(1,:))
sigY = [t;y_sum]
Ycmd = timeseries(sigY(2:end,:),sigY(1,:))
sigX = [t;x_sum]
Xcmd = timeseries(sigX(2:end,:),sigX(1,:))

clear section
clear start_section
clear end_section
clear y_sum
clear z_sum
clear x_sum
clear i
clear k

%%%%%%%%%%%%%%%%%%%%%
% Inicio iteracion  %
%%%%%%%%%%%%%%%%%%%%%

pruebas=100;
EnergiaWh = zeros(pruebas,1);
masabaterias = zeros(pruebas,1);

for p=1:pruebas
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      Iniciar Viento     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = 0:0.1:20;
y1 = wblpdf(x,C,K);
y2 = wblcdf(x,C,K);
r1 = rand(2,1);
a = r1(1);
b = r1(2);
intervalo = [min(r1) max(r1)];
c = (b-a).*rand(2,1)+a;
c_range = [min(c) max(c)];
probabilidad = c_range (2);

vwind = real(-C*(log(1-probabilidad))^(1/K))

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Iniciar Simulink    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim('Viento_coordenadas.slx')


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Funcion baterias    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('matrizcoefpotencia'); %kt
load('matrizcoefempuje'); %b

density = 0.0195;
tipradio =0.15;
%% Creación ded vector b y kt (diagonal de las matrices anteriores)
Cp1 = zeros(length(Cp),1);
Ct1 = zeros(length(Ct),1);

% Vectores diagonales de la matriz (101x1)
for i = 1:length(Cp1)
    Cp1(i,1) = Cp (i,i);
    Ct1(i,1) = Ct (i,i);
end




thrust1 = (0.4:0.1104:11.44)';

%Creación nueca matriz

matrizdecoeficientes = [thrust1 Cp1 Ct1];


%% Cálculo revoluciones
T = ans.Thrust; %vector de tantas filas como sampleos de empuje y una columna
Mp = ans.Mp;
Mq = ans.Mq;
Mr = ans.Mr;


matrizposicionesnueva = zeros(length(T),1);



for i = 1:length(T)
    for j = 1:length(matrizdecoeficientes)
        if (T(i)>=(matrizdecoeficientes(j,1)-0.1) && T(i)<=(matrizdecoeficientes(j,1)+0.1))  
                         
              matrizposicionesnueva(i,1)= j; %Matriz posiciones
        end
    end
 end   
    
Cp2 = zeros(length(T),1);
Ct2 = zeros(length(T),1);

for i = 1:length(T)
    Cp2(i,1) = matrizdecoeficientes(matrizposicionesnueva(i),2);
    Ct2(i,1) = matrizdecoeficientes(matrizposicionesnueva(i),3);
end

for e = 1:length(T)
    
    omega(e,1) = real(sqrt(((T(e))/(4*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)))-(Mq(e)/(2*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)*l))-(Mr(e)/(4*Ct2(e)*density*4*pi^(2)*(2*tipradio)^(4)))));
    omega(e,2) = real(sqrt(((T(e))/(4*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)))-(Mp(e)/(2*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)*l))+(Mr(e)/(4*Ct2(e)*density*4*pi^(2)*(2*tipradio)^(4)))));
    omega(e,3) = real(sqrt(((T(e))/(4*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)))+(Mq(e)/(2*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)*l))-(Mr(e)/(4*Ct2(e)*density*4*pi^(2)*(2*tipradio)^(4)))));
    omega(e,4) = real(sqrt(((T(e))/(4*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)))+(Mp(e)/(2*Cp2(e)*density*(2*pi)^(3)*(2*tipradio)^(5)*l))+(Mr(e)/(4*Ct2(e)*density*4*pi^(2)*(2*tipradio)^(4)))));
        
end

%% Potencia consumida 
Potencia = zeros(length(T),4);
for  i = 1:length(T)

    Potencia (i,1) = density*(2*pi)^(3)*(2*tipradio)^(5)*Cp2(i)*omega(i,1)*omega(i,1)*omega(i,1);
    Potencia (i,2) = density*(2*pi)^(3)*(2*tipradio)^(5)*Cp2(i)*omega(i,2)*omega(i,2)*omega(i,2);
    Potencia (i,3) = density*(2*pi)^(3)*(2*tipradio)^(5)*Cp2(i)*omega(i,3)*omega(i,3)*omega(i,3);
    Potencia (i,4) = density*(2*pi)^(3)*(2*tipradio)^(5)*Cp2(i)*omega(i,4)*omega(i,4)*omega(i,4);

end


Torque = zeros(length(T),4);
for i = 1:length(T)
    
    Torque(i,1) = Potencia(i,1)/omega(i,1);
    Torque(i,2) = Potencia(i,2)/omega(i,2);
    Torque(i,3) = Potencia(i,3)/omega(i,3);
    Torque(i,4) = Potencia(i,4)/omega(i,4);
    
end



%% Energía consumida
Energia(p) = (sum(Potencia(:,1),'omitnan')+sum(Potencia(:,2),'omitnan')+sum(Potencia(:,3),'omitnan')+sum(Potencia(:,4),'omitnan'))*ts; %Ws
EnergiaWh(p) = Energia(p)*0.000277778;

masabaterias(p)= EnergiaWh(p)/densidadbaterias;
end


