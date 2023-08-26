clear, clc

ranuras=255;
max=5;
x=0:1:255;
x=x/256;

v=input('Introduzca tensión inicial: ');
disp(['V = ' num2str(v) 'V']);
min=v*ranuras/max;
md=ranuras-min;
disp(['Min value = ' num2str(min)]);
disp(['Range = ' num2str(md)]);
y=x*md+min;


curva2='{ ';
for k=1:ranuras-1
    curva2 = [curva2, num2str(int16(y(k))), ', '];
end
curva2=[curva2, num2str(int16(y(end))), ' };'];
curva2

vout=input('Introduzca valor de tensión: ');
disp(['Vout = ' num2str(vout) 'V']);
dc=(vout-v)*ranuras/(max-v);
disp(['DC = ' num2str(int16(dc))]);