clc;
clear;
close all

%% Defino constantes del robot

a1 = 0;         d1 = 0.1625;    alpha1 = deg2rad(90);
a2 = -0.425;    d2 = 0;         alpha2 = deg2rad(0);
a3 = -0.3922;   d3 = 0;         alpha3 = deg2rad(0);
a4 = 0;         d4 = 0.1333;    alpha4 = deg2rad(90);
a5 = 0;         d5 = 0.0997;    alpha5 = deg2rad(-90);
a6 = 0;         d6 = 0.0996;    alpha6 = deg2rad(0);
a7 = 0;         d7 = 0.1669;    alpha7 = deg2rad(0);    theta7 = deg2rad(0);

% Posiciones angulares de cada articulación
theta1 = deg2rad(0);
theta2 = deg2rad(-90);
theta3 = deg2rad(-90);
theta4 = deg2rad(0);
theta5 = deg2rad(90);
theta6 = deg2rad(0);

fprintf("Ángulos de cada motor [deg]:\n")
fprintf("Base: %.2fº\n", rad2deg(theta1))
fprintf("Hombro: %.2fº\n", rad2deg(theta2))
fprintf("Codo: %.2fº\n", rad2deg(theta3))
fprintf("Muñeca 1: %.2fº\n", rad2deg(theta4))
fprintf("Muñeca 2: %.2fº\n", rad2deg(theta5))
fprintf("Muñeca 3: %.2fº\n\n", rad2deg(theta6))

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

T67 = [cos(theta7)  -sin(theta7)*cos(alpha7)    sin(theta7)*sin(alpha7)     a7*cos(theta7)
       sin(theta7)  cos(theta7)*cos(alpha7)     -cos(theta7)*sin(alpha7)    a7*sin(theta7)
       0            sin(alpha7)                 cos(alpha7)                 d7
       0            0                           0                           1];

T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;
disp('Matriz T07:')
disp(T07);

%% Extraigo los datos de posición cartesiana
 
X_EF = T07(1,4) * 1000;
Y_EF = T07(2,4) * 1000;
Z_EF = T07(3,4) * 1000;

fprintf('La posición cartesiana del EF es [%.4f, %.4f, %.4f] [mm]\n\n', X_EF, Y_EF, Z_EF);

%% Extraigo el ángulo de TCP

%Por eje-ángulo
ang = rotationToURVector(T07(1:3, 1:3));
fprintf('El vector de rotación es [%.4f, %.4f, %.4f] [grad]\n', rad2deg(ang));

% Función que extrae el ángulo de una matriz de rotación
function ur_rotation_vector = rotationToURVector(R)
% Verificar que la matriz sea válida
    if ~isequal(size(R), [3, 3])
        error('La matriz de rotación debe ser 3x3.');
    end
    % Convertir matriz de rotación a ángulo-eje
    axang = rotm2axang(R);
    axis = axang(1:3);  % Eje de rotación
    angle = axang(4);   % Ángulo de rotación
        
    % Escalar el eje por el ángulo
    ur_rotation_vector = axis * angle;
end