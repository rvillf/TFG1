%% Animación Simulink 3D
figure;
xlimit = [-6 6];
ylimit = [-6 6];
zlimit = [0 8];
width = 600;
height = 600;
NewFigure(xlimit,ylimit,zlimit,-27,27,width,height);
xlabel('Eje X (m)');
ylabel('Eje Y (m)');
zlabel('Eje Z (m)');
%VisAttitude([0,0,0],'black')
%VisAttitude(deg2rad(RefEuler),'g:')
pause(2)
AnimEuler(out.time,out.XYZ,out.EulerAngles,out.VXYZ)

%% Plot XYZ en función del tiempo
figure;
plot(out.time,out.XYZ(:,1),out.time,out.XYZ(:,2),out.time,out.XYZ(:,3));
xlabel('Tiempo (s)');
ylabel('Posición (m)');
legend('Posición en x','Posición en y','Posición en z')
% subplot(3,1,2);
% plot(out.time,out.XYZ(:,2));
% xlabel('Tiempo (s)');
% ylabel('Posición en Y (m)');
% 
% subplot(3,1,3);
% plot(out.time,out.XYZ(:,3));
% xlabel('Tiempo (s)');
% ylabel('Posición en Z (m)');

%% Plot phi, theta y psi en función del tiempo
figure;
plot(out.time,out.EulerAngles(:,1),out.time,out.EulerAngles(:,2),out.time,out.EulerAngles(:,3));
xlabel('Tiempo (s)');
ylabel('Ángulo (rd)');
legend('Ángulo \phi','Ángulo \theta','Ángulo \psi')
% subplot(3,1,2);
% plot(out.time,out.EulerAngles(:,2));
% xlabel('Tiempo (s)');
% ylabel('Ángulo theta (rd)');
% 
% subplot(3,1,3);
% plot(out.time,out.EulerAngles(:,3));
% xlabel('Tiempo (s)');
% ylabel('Ángulo psi (rd)');

%% Plot Empuje(T) y momentos de cabeceo, alabeo y guiñada en función del tiempo
%% Plot phi, theta y psi en función del tiempo
figure;
subplot(4,1,1);
plot(out.time,out.Thrust);
xlabel('Tiempo');
ylabel('Empuje (N)');

subplot(4,1,2);
plot(out.time,out.Mp);
xlabel('Tiempo (s)');
ylabel('Mp (Nm)');

subplot(4,1,3);
plot(out.time,out.Mq);
xlabel('Tiempo (s)');
ylabel('Mq (Nm)');

subplot(4,1,4);
plot(out.time,out.Mr);
xlabel('Tiempo (s)');
ylabel('Mr (Nm)');


%% Funciones locales

function NewFigure(xlim,ylim,zlim,viewx,viewy,w,h)
    set(gca, 'XLim', xlim,'YLim',ylim,'ZLim',zlim);
    view(viewx,viewy)
    x0=10;
    y0=10;
    set(gcf,'position',[x0,y0,w,h])
    hold on;
    grid on;
end

function AnimEuler(t_plot,XYZs,EulerAngles,VXYZs)
    t_section = 0
    curve = animatedline('LineWidth',0.5);
    curveTR = animatedline('LineWidth',1,'LineStyle',':');
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Realizar animación
            Euler = EulerAngles(i,:)
            XYZ = XYZs(i,:);
            VXYZ = VXYZs(i,:);
            
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O;
            
            addpoints(curve, XYZ(1), XYZ(2),XYZ(3));

                 
            line1 = drawline(XYZ,O_I(:,1),'b--',1.5);
            line2 = drawline(XYZ,O_I(:,2),'g--',1.5);
            line3 = drawline(XYZ,O_I(:,3),'r--',1.5);
            line5 = extendline(XYZ,O_I(:,1),'b:');
            
            frame1 = drawline(XYZ,0.5*O_I(:,1)+0.5*O_I(:,2),'black',2.5);
            frame2 = drawline(XYZ,0.5*O_I(:,1)-0.5*O_I(:,2),'black',2.5);
            frame3 = drawline(XYZ,-0.5*O_I(:,1)+0.5*O_I(:,2),'black',2.5);
            frame4 = drawline(XYZ,-0.5*O_I(:,1)-0.5*O_I(:,2),'black',2.5);           

            drawnow;
            pause(0.05);
            
            % logs     
            xlabel(string( num2str(t_plot(i),'%.1f') )+' sec   ');
            
            dispstr7 = string( num2str( VXYZ(1),'%.1f' ) );
            dispstr8 = string( num2str( VXYZ(2),'%.1f' ) );
            dispstr9 = string( num2str( VXYZ(3),'%.1f' ) );
            vstr = 'Velocidad ['+dispstr7+ ' , '+dispstr8 + ' , ' + dispstr9 + ']';
            
            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );
            title('Angulos de Euler ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
                + ' , ' +dispstr5+ ' , '+dispstr6 +']   ' + vstr);
            
            t_section = t_section + 0.2;            

            delete(line1);
            delete(line2);
            delete(line3);
            delete(line5);
                
            delete(frame1);
            delete(frame2);
            delete(frame3);
            delete(frame4);

        end
    end    
end

function SubAnimEuler(t_plot,XYZs,EulerAngles,TR)
    figure;
    t_section = 0;  
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Realizar animación    
            subplot(1,2,1)
            NewFigure([-3 3],[-1 5],[0 6],0,0,1200,600);
            scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')       
            Euler = EulerAngles(i,:);
            XYZ = XYZs(i,:);
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O;
            line1 = drawline(XYZ,O_I(:,1),'b');
            line2 = drawline(XYZ,O_I(:,2),'g');
            line3 = drawline(XYZ,O_I(:,3),'r');
            line4 = extendline(XYZ,O_I(:,3),'r--');
            line5 = extendline(XYZ,O_I(:,1),'b:');
            
            % labels
            title('Plano XZ (vista lateral');
            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );            
            mystr = string( num2str(t_plot(i),'%.1f'))+' sec 3  EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
            + ' , ' +dispstr5+ ' , '+dispstr6 +']';
            txt1 = annotation('textbox', [0.35, 0.9, 0.1, 0.1], 'string', mystr);

            
            subplot(1,2,2);
            NewFigure([-3 3],[-1 5],[0 6],0,90,1200,600);
            scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black');       
            line6 = drawline(XYZ,O_I(:,1),'b');
            line7 = drawline(XYZ,O_I(:,2),'g');
            line8 = drawline(XYZ,O_I(:,3),'r');
            line9 = extendline(XYZ,O_I(:,3),'r--');
            line10 = extendline(XYZ,O_I(:,1),'b:');
            % labels
            title('Plano XY (vista de planta)');

            
            drawnow;
            pause(0.01);        
            if i ~= length(EulerAngles)
                delete(line1);
                delete(line2);
                delete(line3);
                delete(line4);
                delete(line5);
                delete(line6);
                delete(line7);
                delete(line8);
                delete(line9);
                delete(line10);
                delete(txt1);
            end
                        
            t_section = t_section + 0.2;
        end
    end    
end

function m = matrixB2I(phi,theta,psi)
    T_BtoV2 = [[1 0 0];[0 cos(-phi) sin(-phi)];[0 -sin(-phi) cos(-phi)]];
    T_V2toV1 = [[cos(-theta) 0 -sin(-theta)];[0 1 0];[sin(-theta) 0 cos(-theta)]];
    T_V1toI = [[cos(-psi) sin(-psi) 0];[-sin(-psi) cos(-psi) 0];[0 0 1]];
    m = T_V1toI*T_V2toV1*T_BtoV2;
end

function line = drawline(p1,p2,color,width)
    pt1 = p1;
    pt2 = pt1 + transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',width);
end

function line = extendline(p1,p2,color)
    pt1 = p1;
    pt2 = pt1 + 20*transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',1.5);
end

function VisAttitude(Euler,linsty)
    O = eye(3);
    T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
    O_I = T_BtoI*O;
    for i = 1:length(O_I)
        drawline(O_I(:,i),linsty);
    end
    z = O_I(:,3);
    scatter3(z(1),z(2),z(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
end


