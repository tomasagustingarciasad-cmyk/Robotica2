clc, clear, close all;
% Ejecutar el script con los parámetros DH
robot_3gdl;
%% 2. CONFIGURACIÓN INICIAL
q = [pi/4, pi/6, -pi/4, 0]; 

figure('Name', 'Simulación Cinemática Analítica', 'NumberTitle', 'off');
R.plot(q, 'workspace', workspace, 'scale', 0.5);
hold on;

%% 3. BUCLE PRINCIPAL DE CONTROL
disp('--- Bucle de Cinemática Analítica ---');
while true
    % Solicitar datos al usuario
    X_req = input('Ingrese coordenada X (metros) [Dejar vacío y dar Enter para salir]: ');
    
    if isempty(X_req)
        disp('Saliendo de la simulación...');
        break;
    end
    
    Y_req = input('Ingrese coordenada Y (metros): ');
    Z_req = input('Ingrese coordenada Z (metros): ');
    Pitch_req = input('Ingrese la inclinación (Pitch) deseada en GRADOS: ');

    % Llamada a tu propia función matemática (definida al final del script)
   [q_objetivo, alcanzable] = cinematica_analitica(X_req, Y_req, Z_req, Pitch_req, L1, L2, L3, L4);

    % Red de seguridad si la coordenada es físicamente imposible
    if ~alcanzable
        disp(' ');
        disp('⚠️ IMPOSIBLE LLEGAR ALLÍ: El brazo es demasiado corto o la inclinación pedida fuerza una colisión.');
        disp('Intenta con otros valores.');
        disp(' ');
        continue;
    end

    % --- NUEVA FUNCIONALIDAD: Imprimir el vector articular ---
    disp(' ');
    disp('✅ ¡Coordenada alcanzable!');
    
    % Imprimir en Radianes (Formato interno de MATLAB)
    fprintf('Vector articular [q1, q2, q3, q4] (Radianes): [%.4f, %.4f, %.4f, %.4f]\n', ...
            q_objetivo(1), q_objetivo(2), q_objetivo(3), q_objetivo(4));
            
    % Convertir y e imprimir en Grados (Para lectura humana)
    q_deg = q_objetivo * (180 / pi);
    fprintf('Vector articular [q1, q2, q3, q4] (Grados):   [%.1f°, %.1f°, %.1f°, %.1f°]\n', ...
            q_deg(1), q_deg(2), q_deg(3), q_deg(4));
    disp(' ');
    % ---------------------------------------------------------

    % Generación de trayectoria suave en el espacio articular[cite: 2]
    Q_trayectoria = jtraj(q, q_objetivo, 50);

    % Limpiar figura y animar
    hold off;
    R.plot(Q_trayectoria, 'workspace', workspace, 'scale', 1);

    % Guardar la posición final como punto de partida para el próximo movimiento
    q = Q_trayectoria(end, :);
end


