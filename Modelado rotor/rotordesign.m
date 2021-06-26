%% Inputs
%Parámetros físicos
masa = 2;
blades = 2;
segments = 20;
optimuscl = 0.7;
tipradio =0.03:0.005:0.4;
Np = zeros(length(tipradio),1);
for h=1:length(tipradio)
%Parámetros del entorno marciano
cinematicviscosity = 0.00001289;
Mtip = 0.7; 
Rmartian = 188.94;
Tmartian = 231;
density = 0.0195;
omega = 1.288;

surface = pi*tipradio(h)*tipradio(h);
hubradio = 0.05*tipradio(h); 

%Velocidad de giro rotor
angularvelocity = (Mtip*sqrt(Rmartian*omega*Tmartian))/tipradio(h);
angularvelocityrpm = angularvelocity*60/(2*pi);


%% Creación segmentos
utilblade = tipradio(h) - hubradio;
segmentmeasure = utilblade/segments;
halfsegment = segmentmeasure/2;

%% Inicio bucle

%angularvelocity =11:11:1106.45; %rd/s %vector de 101 caracteres
thrust = 2.86;
%thrust =0.1:0.0276:2.86;

%Cp = zeros(length(angularvelocity),length(thrust));
%Ct = zeros(length(angularvelocity),length(thrust));
%Torque = zeros(length(angularvelocity),length(thrust));
division = zeros(segments,1);
for i = 1:segments   
  division (i,1) = hubradio + i*segmentmeasure; 
end
nondimensionalradius = zeros(segments,1);
for i= 1:segments   
        
            nondimensionalradius(i) = division(i)/tipradio(h);
        
end

%for t = 1:length(thrust)


%for v = 1:length(angularvelocity)

    displacementvelocityratio = 0; 
    iteracion = 1;
    while iteracion <=10
        
        iteracion = iteracion+1;

        %% Creación vector segmentos     
        
        
        freestreamvelocity = sqrt(thrust/(2*density*surface));
        
        

        
        speedratio = freestreamvelocity/(tipradio(h)*angularvelocity);

        for i = 1:segments 

        tanphi (i,1) = ((1+(displacementvelocityratio/2))*speedratio)/nondimensionalradius(i);

        end

        phird = atan(tanphi);
        phigr = phird*180/pi;
        phitip = phird(segments);

        for i = 1:segments 

            f(i,1) = (blades/2)*(1-nondimensionalradius(i))/sin(phitip);    

        end

        for i = 1:segments 

        F(i,1) = (2/pi)*acos(exp(-f(i)));  

        end

        for i = 1:segments 

        x(i,1) = nondimensionalradius(i)/speedratio;

        end

        for i = 1:segments
            G(i,1) = F(i)*x(i)*cos(phird(i))*sin(phird(i));
        end
        
        
        for i = 1:segments 
            Wc(i,1) = 4*pi*speedratio*G(i)*freestreamvelocity*tipradio(h)*displacementvelocityratio/(optimuscl*blades);
        end
        Table = readtable('Datosperfil');
        
        Re = Wc/cinematicviscosity;

        %Re = [9000;15000;25000;35000;55000];
        
        %% Seleccionador de columnas
        %Matriz seleccionadora de columnas Cl
        ACl = zeros(length(Re),1); 
        for i = 1:length(Re)
            if Re(i)<=10000
                ACl(i,1) = 2;
            elseif ((Re(i)>=10000) && (Re(i)<20000))
                ACl(i,1) = 3;
            elseif ((Re(i)>=20000) && (Re(i)<30000))
                ACl(i,1) = 4;
            elseif ((Re(i)>=30000) && (Re(i)<40000))
                ACl(i,1) = 5;
            elseif ((Re(i)>=40000) && (Re(i)<50000))
                ACl(i,1) = 6;
            elseif ((Re(i)>=50000) && (Re(i)<60000))
                ACl(i,1) = 7;
            elseif ((Re(i)>=60000) && (Re(i)<70000))
                ACl(i,1) = 8;
            elseif ((Re(i)>=70000) && (Re(i)<80000))
                ACl(i,1) = 9;
            elseif ((Re(i)>=80000) && (Re(i)<90000))
                ACl(i,1) = 10;
            elseif ((Re(i)>=90000) && (Re(i)<100000))
                ACl(i,1) = 11;
            elseif ((Re(i)>=100000) && (Re(i)<110000))
                ACl(i,1) = 12;
            elseif ((Re(i)>=110000) && (Re(i)<120000))
                ACl(i,1) = 13;
            elseif ((Re(i)>=120000) && (Re(i)<130000))
                ACl(i,1) = 14;
            elseif ((Re(i)>=130000) && (Re(i)<140000))
                ACl(i,1) = 15;
            elseif ((Re(i)>=140000) && (Re(i)<150000))
                ACl(i,1) = 16;
            elseif Re(i)>= 150000
                ACl(i,1) = 16;
            end
        end

        %Búsqueda de fila dentro de la columna (Cl)
        BCl = zeros(66,length(Re));
        matrizposiciones = zeros(66,length(Re));
        matrizcl = zeros(66,length(Re));

        for i = 1:length(Re)
          BCl(:,i) = Table.(ACl(i));
            for e = 1:66
                if (BCl(e,i)>=(optimuscl) && BCl(e,i)<=(optimuscl+0.05))
                    if (BCl(e,i)> optimuscl)
                        matrizposiciones (e,i) = e;
                        matrizcl (e,i) = BCl(e,i);
                    end

                end

            end 

        end
        posicion = max(matrizposiciones)';
        vectorcl = max(matrizcl)';
        %Búsqueda del ángulo a partir de las filas anteriormente buscadas (Cl)
        angulo = zeros(length(Re),1);
        columnaangulos = Table.(1);
        for i = 1:length(Re)

           angulo(i,1) = columnaangulos(posicion(i));

        end

        %Matriz seleccionadora de columnas (Cd)
        ACd = zeros(length(Re),1); 
        for i = 1:length(Re)
            if Re(i)<=10000
                ACd(i,1) = 19;
            elseif ((Re(i)>=10000) && (Re(i)<20000))
                ACd(i,1) = 20;
            elseif ((Re(i)>=20000) && (Re(i)<30000))
                ACd(i,1) = 21;
            elseif ((Re(i)>=30000) && (Re(i)<40000))
                ACd(i,1) = 22;
            elseif ((Re(i)>=40000) && (Re(i)<50000))
                ACd(i,1) = 23;
            elseif ((Re(i)>=50000) && (Re(i)<60000))
                ACd(i,1) = 24;
            elseif ((Re(i)>=60000) && (Re(i)<70000))
                ACd(i,1) = 25;
            elseif ((Re(i)>=70000) && (Re(i)<80000))
                ACd(i,1) = 26;
            elseif ((Re(i)>=80000) && (Re(i)<90000))
                ACd(i,1) = 27;
            elseif ((Re(i)>=90000) && (Re(i)<100000))
                ACd(i,1) = 28;
            elseif ((Re(i)>=100000) && (Re(i)<110000))
                ACd(i,1) = 29;
            elseif ((Re(i)>=110000) && (Re(i)<120000))
                ACd(i,1) = 30;
            elseif ((Re(i)>=120000) && (Re(i)<130000))
                ACd(i,1) = 31;
            elseif ((Re(i)>=130000) && (Re(i)<140000))
                ACd(i,1) = 32;
            elseif ((Re(i)>=140000) && (Re(i)<150000))
                ACd(i,1) = 33;
            elseif Re(i)>= 150000
                ACd(i,1) = 33;
            end
        end

        %Búsqueda de fila dentro de la columna (Cl)
        BCd = zeros(66,length(Re));
        vectorcd = zeros(length(Re),1);
        for i = 1:length(Re)

            vectorcd (i,1) = Table.(ACd(i))(posicion(i));


        end

        %% Creación vector drag-to-lift ratio
        dragtoliftratio = vectorcd/vectorcl;
        vectordragtoliftratio = dragtoliftratio (:,1);

        %% Creación a y a'
        a = zeros(length(Re),1);
        aprima = zeros(length(Re),1);

        for i = 1: length(Re)

            a(i,1)= (displacementvelocityratio/2)*(cos(phird(i))^(2))*(1-vectordragtoliftratio(i)*tan(phird(i)));
            aprima(i,1) = (displacementvelocityratio/2*x(i))*cos(phird(i))*sin(phird(i))*(1+vectordragtoliftratio(i)/tan(phird(i)));

        end

        %% Creación de W
        W = zeros(length(Re),1);
        for i = 1:length(Re)
            W(i,1) = freestreamvelocity*(1+a(i))/sin(phird(i));
        end

        %% Creación de la cuerda en cada segmento
        c = zeros(length(Re),1);
        for i = 1:length(Re)
            c(i,1) = Wc(i)/W(i);
        end

        %% Creación "Blade Twist"
        betat = zeros(length(Re),1);
        for i = 1:length(Re)
           betat(i,1) = angulo(i)+(phigr(i));
        end

        %% Creación de I1' e I2'|| J1' y J2'
        I1prima = zeros(length(Re),1);
        I2prima = zeros(length(Re),1);
        J1prima = zeros(length(Re),1);
        J2prima = zeros(length(Re),1);

        for i = 1:length(Re)
           I1prima(i,1) = 4*nondimensionalradius(i)*G(i)*(1-vectordragtoliftratio(i)*tan(phird(i)));
           I2prima(i,1) = speedratio*(I1prima(i)/(2*nondimensionalradius(i)))*(1+vectordragtoliftratio(i)/tan(phird(i)))*sin(phird(i))*cos(phird(i));
           J1prima(i,1) =4*nondimensionalradius(i)*G(i)*(1+vectordragtoliftratio(i)/tan(phird(i)));
           J2prima(i,1) =(J1prima(i)/2)*(1-vectordragtoliftratio(i)*tan(phird(i)))*(cos(phird(i))^(2));
        end


        I1= sum(I1prima);
        I2 = sum(I2prima);
        J1 = sum(J1prima);
        J2 = sum(J2prima);

        %% Nuevo displacement velocity ratio y Tc 
        Tc = 2*thrust/(density*(freestreamvelocity^(2))*pi*(tipradio(h)^(2)));
        displacementvelocityratio = (I1/(2*I2))-sqrt(((I1/(2*I2))^(2))-Tc/I2);

        %%  Nuevo Pc
        Pc = (J1*displacementvelocityratio)+(J2*displacementvelocityratio.^(2));
        %% Potencia, Torque y Rendimiento propulsivo
        P = (Pc*density*(freestreamvelocity^(3))*pi*(tipradio(h)^(2)))/2;
        Torque = P/angularvelocity;
        Np(h,1) = (Tc/Pc)*100;
        
        %% Coeficiente de empuje y de torque 
        Ct = thrust/(density*(angularvelocity/2*pi)^(2)*(2*tipradio(h))^(4));
        Cp = P/(density*(angularvelocity/2*pi)^(3)*(2*tipradio(h))^(5));
        
 end    
%end
% plot(division,c);
% xlabel('Envergadura de la pala (m)');
% ylabel('Cuerda (m)');
%plot(division,betat,division,phigr,division,angulo)
%xlabel('Envergadura de la pala (m)');
%ylabel('Ángulo (º)');
%legend('\beta = \phi+\alpha','\phi','\alpha')
end
plot (tipradio,Np);
xlabel('Radio del rotor (m)');
ylabel('Rendimiento propulsivo (%)');