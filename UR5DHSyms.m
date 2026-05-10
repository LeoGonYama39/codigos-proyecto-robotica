clc;
clear;
close all

% Posiciones angulares de cada articulación
syms theta1 theta2 theta3 theta4 theta5 theta6
pi = sym(pi);

%% Defino constantes del robot

a1 = 0;         d1 = 0.1625;    alpha1 = deg2rad(90);
a2 = -0.425;    d2 = 0;         alpha2 = deg2rad(0);
a3 = -0.3922;   d3 = 0;         alpha3 = deg2rad(0);
a4 = 0;         d4 = 0.1333;    alpha4 = deg2rad(90);
a5 = 0;         d5 = 0.0997;    alpha5 = deg2rad(-90);
a6 = 0;         d6 = 0.0996;    alpha6 = deg2rad(0);


%% CD usando método DH

T01 = [cos(theta1)  -sin(theta1)*cos(alpha1)    sin(theta1)*sin(alpha1)     a1*cos(theta1)
       sin(theta1)  cos(theta1)*cos(alpha1)     -cos(theta1)*sin(alpha1)    a1*sin(theta1)
       0            sin(alpha1)                 cos(alpha1)                 d1
       0            0                           0                           1];

T12 = [cos(theta2)  -sin(theta2)*cos(alpha2)    sin(theta2)*sin(alpha2)     a2*cos(theta2)
       sin(theta2)  cos(theta2)*cos(alpha2)     -cos(theta2)*sin(alpha2)    a2*sin(theta2)
       0            sin(alpha2)                 cos(alpha2)                 d2
       0            0                           0                           1];

T23 = [cos(theta3)  -sin(theta3)*cos(alpha3)    sin(theta3)*sin(alpha3)     a3*cos(theta3)
       sin(theta3)  cos(theta3)*cos(alpha3)     -cos(theta3)*sin(alpha3)    a3*sin(theta3)
       0            sin(alpha3)                 cos(alpha3)                 d3
       0            0                           0                           1];

T34 = [cos(theta4)  -sin(theta4)*cos(alpha4)    sin(theta4)*sin(alpha4)     a4*cos(theta4)
       sin(theta4)  cos(theta4)*cos(alpha4)     -cos(theta4)*sin(alpha4)    a4*sin(theta4)
       0            sin(alpha4)                 cos(alpha4)                 d4
       0            0                           0                           1];

T45 = [cos(theta5)  -sin(theta5)*cos(alpha5)    sin(theta5)*sin(alpha5)     a5*cos(theta5)
       sin(theta5)  cos(theta5)*cos(alpha5)     -cos(theta5)*sin(alpha5)    a5*sin(theta5)
       0            sin(alpha5)                 cos(alpha5)                 d5
       0            0                           0                           1];

T56 = [cos(theta6)  -sin(theta6)*cos(alpha6)    sin(theta6)*sin(alpha6)     a6*cos(theta6)
       sin(theta6)  cos(theta6)*cos(alpha6)     -cos(theta6)*sin(alpha6)    a6*sin(theta6)
       0            sin(alpha6)                 cos(alpha6)                 d6
       0            0                           0                           1];


T06 = T01 * T12 * T23 * T34 * T45 * T56;
disp('Matriz T06:')
TEF = simplify(T06);

%% Extraigo los datos de posición cartesiana
 
X_EF = T06(1,4);
Y_EF = T06(2,4);
Z_EF = T06(3,4);

%% Jacobiano

J = [diff(X_EF, theta1), diff(X_EF, theta2), diff(X_EF, theta3), diff(X_EF, theta4), diff(X_EF, theta5), diff(X_EF, theta6);
     diff(Y_EF, theta1), diff(Y_EF, theta2), diff(Y_EF, theta3), diff(Y_EF, theta4), diff(Y_EF, theta5), diff(Y_EF, theta6);
     diff(Z_EF, theta1), diff(Z_EF, theta2), diff(Z_EF, theta3), diff(Z_EF, theta4), diff(Z_EF, theta5), diff(Z_EF, theta6)];

J = simplify(J);

