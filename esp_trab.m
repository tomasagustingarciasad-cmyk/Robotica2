clc; clear;

robot_3gdl;

% La idea de la primer visualización es recrear el esquema del espacio de
% trabajo que se muestra en la datasheet del robot. Es decir, en el plano
% x1z1. Para ello sólo nos interesa mover las articulaciones 2,3 y 5.
% Para ello seguimos la secuencia especificada en el informe.

% Para la segunda visualización, mostramos una vista superior.

qlim = R.qlim;

N = 10;  % Puntos por segmento para la primera vista
N2 = 20; % Puntos por segmento para la segunda vista

% Límites útiles
q1min = R.qlim(1,1);  q1max = R.qlim(1,2);
q2min = R.qlim(2,1);  q2max = R.qlim(2,2);
q3min = R.qlim(3,1);  q3max = R.qlim(3,2);

% --- REEMPLAZAR DESDE AQUÍ ---
Qall = [];
% Punto inicial: Brazo totalmente estirado
q0 = [0, q2min, 0, 0]; 

% 1. Arco exterior máximo (q3 = 0, q2 barre de min a max)
Qseg = crear_segmento(q0, 2, q2max, N, false);      
Qall = [Qall; Qseg];
qk   = Qseg(end,:);

% 2. Borde inferior (q2 al máximo, q3 pliega a max)
Qseg = crear_segmento(qk, 3, q3max, N, true);           
Qall = [Qall; Qseg];
qk   = Qseg(end,:);

% 3. Arco interior 1 (Brazo plegado hacia arriba, q3 = q3max)
Qseg = crear_segmento(qk, 2, q2min, N, true);       
Qall = [Qall; Qseg];
qk   = Qseg(end,:);

% 4. Borde superior y cruce (q2 al mínimo, q3 va de max a min)
Qseg = crear_segmento(qk, 3, q3min, 2*N, true);       
Qall = [Qall; Qseg];
qk   = Qseg(end,:);

% 5. Arco interior 2 (Brazo plegado hacia abajo, q3 = q3min) -> Zona X negativa
Qseg = crear_segmento(qk, 2, q2max, N, true);       
Qall = [Qall; Qseg];
qk   = Qseg(end,:);

% 6. Cierre del perímetro (Retorno a q3 = 0)
Qseg = crear_segmento(qk, 3, 0, N, true);       
Qall = [Qall; Qseg];
% --- HASTA AQUÍ ---

T  = R.fkine(Qall);      % Obtengo las matrices de TF Homog
P  = transl(T);          % [x y z] para cada punto
XZ = P(:,[1 3]);         % columnas x y z

q_robot = [0, 0, 0, 0];  % Postura estática de referencia
% --- HASTA AQUÍ ---

R.plot(q_robot, 'workspace', workspace, 'view', [0 0]);
view(0,0);

hold on; grid on; axis equal
plot3(XZ(:,1), zeros(size(XZ(:,1))), XZ(:,2), '-o', 'LineWidth', 1.2, 'MarkerSize', 3);
xlabel('X [m]'); zlabel('Z [m]');
title('Vista lateral (plano XZ)');
rotate3d off;


% VISTA SUPERIOR
% Trayectoria: solo q1 varía; articulaciones restantes en extensión máxima
Qtop = repmat([0, 0, 0, 0], N2, 1);
Qtop(:,1) = linspace(q1min, q1max, N2)';

% Inyectar analíticamente la restricción en la vista superior
Qtop(:,4) = -(Qtop(:,2) + Qtop(:,3));

Ttop = R.fkine(Qtop);
Ptop = transl(Ttop);   % [x y z]
XY   = Ptop(:, 1:2);   % [x y]

% Figura nueva con el robot y la trayectoria, vista superior
figure('Color','w');
R.plot(q_robot, 'workspace', workspace, 'view', [0 90]);


view(0,90);
hold on; grid on; axis equal;
rotate3d off;
set(gca, 'CameraViewAngleMode','manual');

plot3(XY(:,1), XY(:,2), zeros(size(XY,1),1), 'r-o', 'LineWidth',1.2, 'MarkerSize',3);

xlabel('X [m]'); ylabel('Y [m]');
title('Vista superior (plano XY)');

% --- REEMPLAZAR DESDE AQUÍ ---
function Qsegmentos = crear_segmento(q_inicial, indice_art, q_final, N, omitir_primero)
    % Inicializamos la matriz con el valor inicial repetido N veces
    Qsegmentos = repmat(q_inicial, N, 1);
    
    % Reemplazamos la columna de la articulación iterada
    Qsegmentos(:, indice_art) = linspace(q_inicial(indice_art), q_final, N)';
    
    % RESTRICCIÓN DE PARALELOGRAMO: Sobrescribe siempre la articulación 4
    Qsegmentos(:, 4) = -(Qsegmentos(:, 2) + Qsegmentos(:, 3));
    
    if omitir_primero
        Qsegmentos = Qsegmentos(2:end, :);
    end
end
% --- HASTA AQUÍ ---