%% Resetear la simulación
resetSim

clc
clear

%% Banda transportadora
% No puedo guardar dos robots (la banda cuenta como robot), entonces 
% siempre al iniciar la simulación, poner una banda en -2400, -300, -350

%% Parámetros DH para UR5e
alp0 = 0;     alp1 = pi/2;  alp2 = 0;
alp3 = 0;     alp4 = pi/2;  alp5 = -pi/2;
alp6 = 0;

d1 = 0.1625;  d2 = 0;   d3 = 0;
d4 = 0.1333; d5 = 0.0997;
d6 = 0.0996;  d7 = 0.1669;

a0 = 0; a1 = 0;     a2 = -0.425;
a3 = -0.3922;      a4 = 0;
a5 = 0;   a6 = 0;

alp = [alp0, alp1, alp2, alp3, alp4, alp5, alp6];
d = [d1, d2, d3, d4, d5, d6, d7];
a = [a0, a1, a2, a3, a4, a5, a6];

%% Algunas matrices de la pose

home = [0, -90, -90, 0, 90, 0];

readyBot = [ 0.000000,     1.000000,     0.000000,  -499.998780 ;
             1.000000,     0.000000,     0.000000,  -0.001206 ;
             0.000000,     0.000000,    -1.000000,   540.965000 ;
             0.000000,     0.000000,     0.000000,   1.000000 ];


pickBot = [0.000000,     1.000000,     0.000000,  -500.000689 ;
           1.000000,     0.000000,     0.000000,   0.000739 ;
           0.000000,     0.000000,    -1.000000,   190.965000 ;
           0.000000,     0.000000,     0.000000,   1.000000 ];

poseInter = [-0.999572,     0.020599,    -0.020767,    20.147554 ;
              0.029250,     0.706518,    -0.707090,  -499.548022 ;
              0.000107,    -0.707395,    -0.706818,   541.118414 ;
              0.000000,     0.000000,     0.000000,     1.000000 ];

%% Calcular las q
for i = 1:10
botInit{i} = gradosURgeom(alp, d, a, getMatrizHomBotella(i, 1));
bot{i} = gradosURgeom(alp, d, a, getMatrizHomBotella(i, 2));
end

poseInter = gradosURgeom(alp, d, a, poseInter);

%% Mover robot
RDK = Robolink; %Hacemos la conexión Matlab-RoboDK
path = RDK.getParam('PATH_LIBRARY'); %Configuración de librería
RDK.ItemList(); %Para visualizar todos los elementos disponibles en RoboDK
robot=RDK.Item('UR5e'); %Creamos el objeto robot en matlab de la clase UR3 
fprintf('Robot configurado:\t %s \n', robot.Name()) 
gripper = RDK.Item('OnRobot SG-a Soft Gripper', RDK.ITEM_TYPE_TOOL);

rutaBotellaB = fullfile(fileparts(mfilename('fullpath')), 'obj', 'BotellaBien.sld');
robot.MoveJ(home);

for i = 1:length(bot)
    botella(i) = generarBotella(rutaBotellaB, RDK);
    movBotella(botella(i));
    pause(1)
    
    robot.MoveJ(readyBot);
    pause(1)
    
    robot.MoveL(pickBot);
    attached = gripper.AttachClosest();
    pause(1);
    
    robot.MoveL(readyBot);
    pause(1);
    
    if i == 1
        robot.MoveJ(poseInter);
    end
    
    robot.MoveJ(botInit{i});
    pause(1)
    
    robot.MoveL(bot{i})
    gripper.DetachAll();
    pause(1)
    
    robot.MoveL(botInit{i})
    pause(1)

end



robot.MoveJ(home);

%% Función para normalizar 
function degNormal = normDegr(deg)
    if(deg > 0)
        degNormal = deg - 360;
    else
        degNormal = deg + 360;
    end
end

%% Función de CI geométrica para UR
function q_grados = gradosURgeom(alp, d, a, H)
alp0 = alp(1);     alp1 = alp(2);  alp2 = alp(3);
alp3 = alp(4);     alp4 = alp(5);  alp5 = alp(6);
alp6 = alp(7);

d1 = d(1);  d2 = d(2); d3 = d(3);
d4 = d(4);  d5 = d(5);
d6 = d(6);  d7 = d(7);

a0 = a(1); a1 = a(2); a2 = a(3);
a3 = a(4); a4 = a(5);
a5 = a(6); a6 = a(7);

nx = H(1, 1);   ox = H(1, 2);   ax = H(1, 3);   px = H(1, 4)/1000;
ny = H(2, 1);   oy = H(2, 2);   ay = H(2, 3);   py = H(2, 4)/1000;
nz = H(3, 1);   oz = H(3, 2);   az = H(3, 3);   pz = H(3, 4)/1000;

q(1) = atan2(py - ay*d6, px - ax*d6) - atan2(-d4, +sqrt((px - ax*d6)^2 + (py - ay*d6)^2 - (-d4)^2));
q_grados(1) = rad2deg(q(1));

%Operaciones preliminares a q5
S5 = +sqrt((-sin(q(1))*nx + cos(q(1))*ny)^2 + (-sin(q(1))*ox + cos(q(1))*oy)^2);
S6 = (-sin(q(1))*ox + cos(q(1))*oy)/S5;
C6 = (sin(q(1))*nx - cos(q(1))*ny)/S5;

%Cálculo de q5
q(5) = atan2(S5, sin(q(1))*ax - cos(q(1))*ay);
q_grados(5) = rad2deg(q(5));

%Cálculo de q6
q(6) = atan2(((-sin(q(1))*ox + cos(q(1))*oy)/(S5)), ((sin(q(1))*nx - cos(q(1))*ny)/(S5)));
q_grados(6 ) =rad2deg(q(6));

%Operaciones preliminares
q234 = atan2((-az/S5), (-(cos(q(1))*ax + sin(q(1))*ay)/(S5)));
q234_grad = deg2rad(q234);
B1 = (cos(q(1))*px + sin(q(1))*py - d5*sin(q234) + d6*cos(q234)*S5);
B2 = (pz - d1 + d5*cos(q234) + d6*sin(q234)*S5);
A = -2*(pz - d1 + d5*cos(q234) + d6*sin(q234)*S5)*a2;

% A_2= -2*B2*a2
B= 2*(cos(q(1))*px + sin(q(1))*py - d5*sin(q234) + d6*cos(q234)*S5)*a2;

% B_2= 2*B1*a2 ;
C= +B1^2 + B2^2 + a2^2 - a3^2;

%Cálculo de q2
q(2) = atan2(B,A) - atan2(C, +sqrt(A^2 + B^2 - C^2));
q_grados(2) = rad2deg(q(2));
q23 = atan2((B2 - a2*sin(q(2)))/(a3), (B1 - a2*cos(q(2)))/a3);

q(4)=q234-q23-2*pi;
%q(4)= q234 - q23;
q_grados(4) = rad2deg(q(4));
q(3) = q23 - q(2) - 2*pi;
q_grados(3) = rad2deg(q(3));


for i = 1:length(q_grados)
    if(q_grados(i) < -360.00) || (q_grados(i) > 360)
        q_grados(i) = normDegr(q_grados(i));
    end
end

end

%% Función para obtener la matriz homogénea de la posición de las botellas en la caja
% modo 1 para obtener la posición arriba de la caja
% modo 2 para obtener la posición en la caja
% numBotella es el número de la botella

function matrizHom = getMatrizHomBotella(numBotella, modo)

    if modo == 1
        switch numBotella
            case 1
                matrizHom = [ 1.000000,     0.000000,     0.000000,   110.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 2
                matrizHom = [ 1.000000,     0.000000,     0.000000,    35.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 3
                matrizHom = [ 1.000000,     0.000000,     0.000000,   -40.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 4
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -120.000000 ;
                             -0.000000,    -1.000000,     0.000000,   500.000000 ;
                             -0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 5
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -200.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 6
                matrizHom = [ 1.000000,     0.000000,     0.000000,   110.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 7
                matrizHom = [ 1.000000,     0.000000,     0.000000,    35.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 8
                matrizHom = [ 1.000000,     0.000000,     0.000000,   -40.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 9
                matrizHom = [ 1.000000,     0.000000,    -0.000000,  -120.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 10
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -200.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   500.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            otherwise
                error('numBotella tiene que estar entre 1 a 10')
        end
    elseif modo == 2
        switch numBotella
            case 1
                matrizHom = [ 1.000000,     0.000000,     0.000000,   110.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 2
                matrizHom = [ 1.000000,     0.000000,     0.000000,    35.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,    -0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 3
                matrizHom = [ 1.000000,     0.000000,     0.000000,   -40.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 4
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -120.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 5
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -200.000000 ;
                              0.000000,    -1.000000,     0.000000,   500.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 6
                matrizHom = [ 1.000000,     0.000000,     0.000000,   110.000000 ;
                             -0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 7
                matrizHom = [ 1.000000,    -0.000000,     0.000000,    35.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 8
                matrizHom = [ 1.000000,    -0.000000,     0.000000,   -40.000000 ;
                             -0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 9
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -120.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            case 10
                matrizHom = [ 1.000000,     0.000000,     0.000000,  -200.000000 ;
                              0.000000,    -1.000000,     0.000000,   600.000000 ;
                              0.000000,     0.000000,    -1.000000,   230.000000 ;
                              0.000000,     0.000000,     0.000000,     1.000000 ];

            otherwise
                error('numBotella tiene que estar entre 1 a 10')
        end
    else
        error('modo tiene que ser 1 o 2')
    end
    sumaTCP = 166.935;
    matrizHom(3, 4) = matrizHom(3, 4) + sumaTCP;
end


%% Función para mover una botella
% Recibe el identificador de una botella para moverla, simula que se mueve
% por la cinta
% Asume que la botella ya está en la posición inicial de la banda
% Ya tiene que estar conectado a RoboDK

function movBotella(botella)

    t = 1:50;

    dist = -2300 + 500; % Calcular distancia a recorrer, de -2300 a -500
    paso = dist / length(t);

    for i = 1:length(t)
    
        current_pose = botella.Pose();
        movement = transl(-paso, 0, 0);
        new_pose = current_pose * movement;
        botella.setPose(new_pose);
    end
end

%% Función para generar una botella en el inicio de la banda
% Recibe la ruta de la botella que quiero poner
% Regresa el apuntador de la botella, que ya queda seteada al inicio de la
% banda

function botella = generarBotella(ruta, RDK)

   aux = RDK.AddFile(ruta);

   current_pose = aux.Pose();
   movement = transl(-2300, 0, 0);
   new_pose = current_pose * movement;
    
   aux.setPose(new_pose);

   botella = aux;

end
