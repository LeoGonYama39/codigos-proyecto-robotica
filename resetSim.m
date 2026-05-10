clc
clear

home = [0, -90, 0, -90, 0, 0];
nombreObj  = {'botella', 'bottle'};

RDK = Robolink; %Hacemos la conexión Matlab-RoboDK
path = RDK.getParam('PATH_LIBRARY'); %Configuración de librería
RDK.ItemList(); %Para visualizar todos los elementos disponibles en RoboDK
robot=RDK.Item('UR5e'); %Creamos el objeto robot en matlab de la clase UR3 

robot.MoveJ([0, -90, -90, 0, 90, 0]);


allObj = RDK.ItemList();          % todos los items de la estacion

for k = 1:numel(allObj)
    item  = allObj{k};
    nombre = lower(item.Name());
    
    isBotella = false;
    for p = 1:numel(nombreObj)
        if contains(nombre, nombreObj{p})
            isBotella = true;
            break;
        end
    end
    
    if isBotella
        item.Delete();
    end
end

disp('Simulación reseteada')