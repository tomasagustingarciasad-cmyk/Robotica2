% Definición del ROBOT 3 GDL de la imagen
clc, clear, close all;

L1 = 0.0; % Altura desde la base hasta la segunda articulación
L2 = 0.15; % Longitud del brazo principal
L3 = 0.150; % Longitud del antebrazo
L4 = 0.050; % Longitud del efector final (Reemplazar por valor real)
% Tabla DH Estándar: [theta, d, a, alpha, sigma]
% Articulación 1: Rotación en Z de la base
% Articulación 2: Rotación en Y (pitch), requiere rotar el eje Z previo con alpha = pi/2
% Articulación 3: Rotación en Y (pitch), ejes Z paralelos, alpha = 0
% Articulación 4: Efector final, ejes Z paralelos, alpha = 0
dh = [
    0, L1,  0, pi/2, 0;
    0,  0, L2,    0, 0;
    0,  0, L3,    0, 0;
    0,  0, L4,    0, 0
];
R = SerialLink(dh, 'name', 'Brazo 4 GDL');
% Límites de cada articulación (Asignación preliminar)
R.qlim(1,1:2) = [-180, 180] * pi/180;
R.qlim(2,1:2) = [-90,   90] * pi/180; 
R.qlim(3,1:2) = [-90,   90] * pi/180;
R.qlim(4,1:2) = [-180, 180] * pi/180;
% Offset 
% Es probable que requieras un offset en q2 y q3 dependiendo de dónde 
% consideres el "cero" físico del robot (ej. brazo totalmente estirado vs posición de reposo).
R.offset = [0, 0, 0, 0]; 
% base y tool
R.base = transl(0, 0, 0.05); 
R.tool = transl(0, 0, 0);

% Espacio de trabajo adaptado a las proporciones [Xmin Xmax Ymin Ymax Zmin Zmax]
workspace = [-0.5, 0.5, -0.5, 0.5, 0, 0.6];