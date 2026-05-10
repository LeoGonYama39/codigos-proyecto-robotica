clear all
close all
clc

%El espacio de trabajo del robot 2DOF es un círculo de Diámetro 0.38

%% Trayectorias para x 
%Condiciones iniciales y finales
xo=500.00; vo=0; aco=0; 
xf=230.00; vf=0; acf=0;
to=13.0; tf=16.0; h=0.001;

%Matriz de tiempos
A=[to^5 to^4 to^3 to^2 to 1;...
    5*to^4 4*to^3 3*to^2 2*to 1 0;...
    20*to^3 12*to^2 6*to 2 0 0;...
    tf^5 tf^4 tf^3 tf^2 tf 1;...
    5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;...
    20*tf^3 12*tf^2 6*tf 2 0 0];

%Vector de condiciones
B=[xo;vo;aco;xf;vf;acf];

%X=A\B;% es lo mismo que X=inv(A)*B Calcula inversa de A y multiplica por B
X=inv(A)*B;
a5=X(1); a4=X(2); a3=X(3); a2=X(4); a1=X(5); a0=X(6);

t=[to:h:tf];

xd=a5*t.^5 + a4*t.^4 + a3*t.^3 + a2*t.^2 + a1.*t + a0;
vd=a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
ad=2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;

syms t

eq=a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0;