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
T = out.Thrust; %vector de tantas filas como sampleos de empuje y una columna
Mp = out.Mp;
Mq = out.Mq;
Mr = out.Mr;


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
Energia = (sum(Potencia(:,1),'omitnan')+sum(Potencia(:,2),'omitnan')+sum(Potencia(:,3),'omitnan')+sum(Potencia(:,4),'omitnan'))*ts; %Ws
EnergiaWh = Energia*0.000277778
densidadbaterias = 161; %Wh/kg
masabaterias = EnergiaWh/densidadbaterias

%% Representación 

figure;
subplot(4,1,1);
plot(out.time,omega(:,1));
xlabel('Tiempo (s)');
ylabel('Omega 1 (rd/s)');

subplot(4,1,2);
plot(out.time,omega(:,2));
xlabel('Tiempo (s)');
ylabel('Omega 2 (rd/s)');

subplot(4,1,3);
plot(out.time,omega(:,3));
xlabel('Tiempo (s)');
ylabel('Omega 3 (rd/s)');

subplot(4,1,4);
plot(out.time,omega(:,4));
xlabel('Tiempo (s)');
ylabel('Omega 4 (rd/s)');
