
x = 0:0.1:20;
y1 = wblpdf(x,C,K);
y2 = wblcdf(x,C,K);

% figure(1);
% plot(x,y1)
% xlabel('Velocidad viento (m/s)');
% ylabel('Probabilidad');
% figure(2);
% plot(x,y2)
% xlabel('Velocidad viento (m/s)');
% ylabel('Probabilidad acumulada');

r1 = rand(2,1);
a = r1(1);
b = r1(2);
intervalo = [min(r1) max(r1)];
c = (b-a).*rand(2,1)+a;
c_range = [min(c) max(c)];
probabilidad = c_range (2);

vwind = real(-C*(log(1-probabilidad))^(1/K))









