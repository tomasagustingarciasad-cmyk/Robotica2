clc, clear, close all;

% 1. Cargar la definición del robot (Tabla DH, límites, etc.)
robot_3gdl;

% 2. Vector de posición articular inicial (Obligatorio 4 elementos)
q = [0, 0, 0, 0]; 

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
    Pitch_req = input('Ingrese la inclinación (Pitch) deseada del efector (grados): ');

    % Validación de alcance teórico para advertir sobre singularidades
    distancia = sqrt(X_req^2 + Y_req^2 + (Z_req - 0.05)^2);
    if distancia > (L2 + L3 + L4)
        warning('Alerta: La coordenada excede el alcance máximo. El solver numérico probablemente fallará.');
    end

% 1. Rotación de la base hacia el objetivo
    q1_req = atan2(Y_req, X_req);
    
    % 2. Convertir el Pitch global requerido a radianes
    phi = Pitch_req * (pi/180);
    
    % 3. Calcular distancia radial y altura compensando el pedestal
    r_req = sqrt(X_req^2 + Y_req^2);
    z_req = Z_req - 0.05; 
    
    % 4. Desacoplo: Ubicación obligatoria de la muñeca para lograr ese Pitch
    r_wrist = r_req - L4 * cos(phi);
    z_wrist = z_req - L4 * sin(phi);
    
    % 5. Resolución trigonométrica para el codo (q3) y el hombro (q2)
    D = (r_wrist^2 + z_wrist^2 - L2^2 - L3^2) / (2 * L2 * L3);
    
    if abs(D) > 1
        disp('Error: La inclinación exigida empuja la muñeca fuera del área física alcanzable. Reintente.');
        continue;
    end
    
    % Configuración "Codo arriba" garantizada
    q3_req = atan2(-sqrt(1 - D^2), D); 
    q2_req = atan2(z_wrist, r_wrist) - atan2(L3 * sin(q3_req), L2 + L3 * cos(q3_req));
    
    % 6. El ángulo del efector compensa a los anteriores para fijarse respecto a la base
    q4_req = phi - q2_req - q3_req;
    
    % Ensamblar el vector objetivo final
    q_objetivo = [q1_req, q2_req, q3_req, q4_req];
    
    % 7. Generación de trayectoria en el espacio articular (50 pasos)
    Q_trayectoria = jtraj(q, q_objetivo, 50);

    % 6. Limpiar la figura de ejes estáticos previos y animar el movimiento
    hold off;
    R.plot(Q_trayectoria, 'workspace', workspace, 'scale', 0.5);

    % 7. Actualizar el estado actual del robot para permitir movimientos sucesivos
    q = Q_trayectoria(end, :);
end