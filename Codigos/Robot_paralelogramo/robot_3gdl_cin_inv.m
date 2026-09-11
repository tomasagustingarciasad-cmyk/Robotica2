clc, clear, close all;

% 1. Cargar la definición del robot (Tabla DH, límites, etc.)
robot_3gdl;

% 2. Vector de posición articular inicial (Obligatorio 4 elementos)
q = [pi/4, pi/6, -pi/4, 0]; 

% 3. Configuración inicial de la figura y ploteo base
figure('Name', 'Simulación Cinemática Inversa', 'NumberTitle', 'off');
R.plot(q, 'workspace', workspace, 'scale', 0.5);
hold on;

% 4. Cálculo explícito de la Cinemática Directa para obtener la matriz 'T'
[T, all] = R.fkine(q);

% Graficar el sistema de referencia de la herramienta final (Tool Center Point)
trplot(T, 'length', 0.1, 'frame', num2str(R.n), 'color', 'r');
title('Brazo 4 GDL - Control Espacial');
grid on;

% --- RESOLUCIÓN CONTINUA DE CINEMÁTICA INVERSA ---
disp('--- Bucle de Cinemática Inversa ---');
while true
    % Solicitar coordenadas al usuario mediante consola
    % Solicitar coordenadas al usuario mediante consola
    X_req = input('Ingrese coordenada X deseada (metros) [Dejar vacío y dar Enter para salir]: ');
    
    % Condición de salida del bucle
    if isempty(X_req)
        disp('Saliendo de la simulación...');
        break;
    end
    
    Y_req = input('Ingrese coordenada Y deseada (metros): ');
    Z_req = input('Ingrese coordenada Z deseada (metros): ');

    % Validación de alcance teórico para advertir sobre singularidades
    distancia = sqrt(X_req^2 + Y_req^2 + (Z_req - 0.05)^2);
    if distancia > (L2 + L3 + L4)
        warning('Alerta: La coordenada excede el alcance máximo. El solver numérico probablemente fallará.');
    end

    % 1. Definir la matriz de transformación del objetivo
    T_objetivo = transl(X_req, Y_req, Z_req); 

    % 2. Vector de máscara [X Y Z Roll Pitch Yaw]
    M = [1 1 1 0 0 0];

    % 3. Cálculo numérico iterativo para encontrar los ángulos finales
    q_objetivo = R.ikine(T_objetivo, 'q0', q, 'mask', M);

    % 4. Generación de trayectoria en el espacio articular (50 pasos)
    Q_trayectoria = jtraj(q, q_objetivo, 50);

    % 5. Imposición analítica del paralelogramo durante toda la trayectoria
    Q_trayectoria(:, 4) = -(Q_trayectoria(:, 2) + Q_trayectoria(:, 3));

    % 6. Limpiar la figura de ejes estáticos previos y animar el movimiento
    hold off;
    R.plot(Q_trayectoria, 'workspace', workspace, 'scale', 0.5);

    % 7. Actualizar el estado actual del robot para permitir movimientos sucesivos
    q = Q_trayectoria(end, :);
end
% --- FIN DEL ARCHIVO ---