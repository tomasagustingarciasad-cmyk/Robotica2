% PI7_CIN_TRAYECTORIA_SCRIPT  Primera iteración de simulación de poses.
%
% QUE HACE : Evalúa las poses teóricas de vuelco y captura, selecciona la 
%            rama cinemática válida priorizando codo arriba, mapea a 
%            actuadores y genera una trayectoria articular inicial.
% RECIBE   : Nada.
% DEVUELVE : Nada. Imprime resultados de validación en consola.
% DEPENDE  : PI7_gen_parametros_params, PI7_cin_inversa_cerrada_fun,
%            PI7_gen_mapeo_actuadas_fun, PI7_cin_directa_fun.
% AUTOR    : Equipo PI7 - Robotica II, UNCuyo - 2026-09-05
% -------------------------------------------------------------------------

clc, clear, close all
P = PI7_gen_parametros_params();

% 1. DEFINICIÓN DE POSES OBJETIVO [x, y, z, phi]
pose_vuelco = [0.200, 0.000, 0.250, deg2rad(-45)];
pose_captura = [0.000, -0.200, 0.15, deg2rad(0)];
poses_mision = [pose_vuelco; pose_captura];
N_poses = size(poses_mision, 1);

fprintf('--- EVALUACIÓN CINEMÁTICA INVERSA ---\n');
[Q_todas, info_CI] = PI7_cin_inversa_cerrada_fun(P, poses_mision, true);

fprintf('\n--- MOTIVOS DE DESCARTE ---\n');
disp(info_CI.motivo);

if any(info_CI.sin_solucion)
    error('PI7:Simulacion:SinSolucion', 'Al menos una pose no tiene solución válida.');
end

% 2. FILTRADO Y SELECCIÓN DE RAMA
q_obj = zeros(N_poses, 4);

for k = 1:N_poses
    % Descartamos las ramas 3 y 4 (base volteada)
    ramas_frontales = info_CI.valida(1:2, k);
    
    if ~any(ramas_frontales)
        error('PI7:Simulacion:RamaFrontal', ...
              'La pose %d exige operar con la base volteada.', k);
    end
    
    % Prioridad: Rama 2 (codo arriba). Fallback: Rama 1 (codo abajo).
    if ramas_frontales(2)
        r_sel = 2;
        fprintf('Pose %d: Seleccionada Rama 2 (codo arriba).\n', k);
    else
        r_sel = 1;
        fprintf('Pose %d: Rama 2 inválida. Forzando Rama 1 (codo abajo).\n', k);
    end
    
    q_obj(k, :) = Q_todas(r_sel, :, k);
end

% 3. MAPEO A COORDENADAS ACTUADAS Y VERIFICACIÓN FÍSICA
% Extraemos la matriz de mapeo inverso (dqa_dq) llamando con ceros
[~, ~, ~, m0] = PI7_gen_mapeo_actuadas_fun(P, zeros(1,4));

qa_obj = zeros(N_poses, m0.na); % Correccion: usamos m0.na devuelto por el mapeo
fprintf('\n--- VERIFICACIÓN DE ACTUADORES ---\n');

for k = 1:N_poses
    % q -> qa (Aplicando el offset si existiese)
    qa_obj(k, :) = q_obj(k, :) * m0.dqa_dq.' - m0.offset_act_rad;
    
    % Verificamos que la coordenada actuada no viole topes físicos ni rango de servo
    [~, ~, ~, im] = PI7_gen_mapeo_actuadas_fun(P, qa_obj(k, :));
    
    if ~im.ok
        fprintf('ERROR Pose %d: Violación de topes actuados o cabeceo.\n', k);
        disp(im.omitidas);
    else
        fprintf('Pose %d: Actuadores y cabeceo dentro del rango seguro.\n', k);
    end
end

% 4. TRAYECTORIA INICIAL (Lazo abierto)
% Usamos jtraj para conectar vuelco y captura en P.sim.n_muestras pasos
t = linspace(0, 2, P.sim.n_muestras)'; % 2 segundos arbitrarios por ahora
[qa_traj, qda_traj, qdda_traj] = jtraj(qa_obj(1,:), qa_obj(2,:), t);

% Convertimos la trayectoria actuada a articular para verificarla
[q_traj, ~, ~, info_traj] = PI7_gen_mapeo_actuadas_fun(P, qa_traj);

if ~info_traj.ok
    warning('La interpolación atraviesa zonas inválidas del espacio actuado.');
end
% Mapeo de velocidades y aceleraciones actuadas a articulares
    qd_traj  = qda_traj * m0.dq_dqa.';
    qdd_traj = qdda_traj * m0.dq_dqa.';

    figure('Name', 'Cinemática Articular', 'NumberTitle', 'off');

    subplot(3,1,1);
    plot(t, q_traj, 'LineWidth', 1.5);
    grid on; title('Posición articular');
    xlabel('Tiempo [s]'); ylabel('Ángulo [rad]');
    legend('q_1 (Base)', 'q_2 (Hombro)', 'q_3 (Codo relativo)', 'q_4 (Muñeca)', 'Location', 'best');

    subplot(3,1,2);
    plot(t, qd_traj, 'LineWidth', 1.5);
    grid on; title('Velocidad articular');
    xlabel('Tiempo [s]'); ylabel('Velocidad [rad/s]');

    subplot(3,1,3);
    plot(t, qdd_traj, 'LineWidth', 1.5);
    grid on; title('Aceleración articular');
    xlabel('Tiempo [s]'); ylabel('Aceleración [rad/s^2]');


% 5. VERIFICACIÓN FINAL CON CINEMÁTICA DIRECTA
[T_traj, info_CD] = PI7_cin_directa_fun(P, q_traj);
fprintf('\nTrayectoria generada: %d muestras. Error máximo CD vs CI: %.2e m\n', ...
        P.sim.n_muestras, max(info_CI.err_pos_max));

% --- NUEVO: CÁLCULO Y GRÁFICAS CARTESIANAS ---
    % 1. Extracción de posición y paso de tiempo
    P_cart = info_CD.p;
    dt_muestreo = t(2) - t(1);

    % 2. Derivación numérica (pierden 1 y 2 muestras)
    V_cart = diff(P_cart) / dt_muestreo;
    A_cart = diff(V_cart) / dt_muestreo;

    % 3. Gráficas
    figure('Name', 'Cinemática Cartesiana', 'NumberTitle', 'off', 'Units', 'normalized');

    % --- POSICIÓN ---
    subplot(3,1,1);
    hold on;
    plot(t, P_cart(:,1), 'LineWidth', 1.5, 'LineStyle', '-', 'DisplayName', 'X');
    plot(t, P_cart(:,2), 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Y');
    plot(t, P_cart(:,3), 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', 'Z');
    hold off;
    grid on; title('Posición cartesiana del efector final');
    xlabel('Tiempo [s]'); ylabel('Coordenadas [m]');
    legend('Location', 'best');

    % --- VELOCIDAD ---
    subplot(3,1,2);
    hold on;
    plot(t(1:end-1), V_cart(:,1), 'LineWidth', 1.5, 'LineStyle', '-', 'DisplayName', 'V_x');
    plot(t(1:end-1), V_cart(:,2), 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'V_y');
    plot(t(1:end-1), V_cart(:,3), 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', 'V_z');
    hold off;
    grid on; title('Velocidad cartesiana del efector final');
    xlabel('Tiempo [s]'); ylabel('Velocidad [m/s]');
    legend('Location', 'best');

    % --- ACELERACIÓN ---
    subplot(3,1,3);
    hold on;
    plot(t(1:end-2), A_cart(:,1), 'LineWidth', 1.5, 'LineStyle', '-', 'DisplayName', 'a_x');
    plot(t(1:end-2), A_cart(:,2), 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'a_y');
    plot(t(1:end-2), A_cart(:,3), 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', 'a_z');
    hold off;
    grid on; title('Aceleración cartesiana del efector final');
    xlabel('Tiempo [s]'); ylabel('Aceleración [m/s^2]');
    legend('Location', 'best');

% --- GRÁFICAS DE COORDENADAS ACTUADAS ---
    figure('Name', 'Cinemática Actuada', 'NumberTitle', 'off');

    subplot(3,1,1);
    plot(t, qa_traj, 'LineWidth', 1.5);
    grid on; title('Posición de actuadores');
    xlabel('Tiempo [s]'); ylabel('Ángulo [rad]');
    legend(m0.nombres, 'Location', 'best');

    subplot(3,1,2);
    plot(t, qda_traj, 'LineWidth', 1.5);
    grid on; title('Velocidad de actuadores');
    xlabel('Tiempo [s]'); ylabel('Velocidad [rad/s]');

    subplot(3,1,3);
    plot(t, qdda_traj, 'LineWidth', 1.5);
    grid on; title('Aceleración de actuadores');
    xlabel('Tiempo [s]'); ylabel('Aceleración [rad/s^2]');


% 6. CONSTRUCCIÓN DEL MODELO Y ANIMACIÓN VISUAL
fprintf('\n--- CONSTRUCCIÓN Y ANIMACIÓN ---\n');
[R, avisos] = PI7_gen_construir_robot_fun(P);

% Imprimir avisos del constructor (requisito de interfaz del proyecto)
cellfun(@(t) fprintf('  [aviso] %s\n', t), avisos);

% Graficar y animar
figure('Name', 'Trayectoria PI7', 'NumberTitle', 'off');
R.plot(q_traj, 'trail', 'r-', 'fps', 30);


% 7. CÁLCULO DE TORQUE ESTÁTICO (Validación manual)
fprintf('\n--- TORQUES ESTÁTICOS (POSTURA DE REFERENCIA) ---\n');
q_ref = [0, 0, 0, 0]; % Brazo horizontal extendido

% Torques en coordenadas articulares (gravedad sobre brazo + esfera)
tau_articular = R.gravload(q_ref);

% Mapeo a torques exigidos a los motores (Caso B: codo L1_base)
    [~, ~, tau_actuador, ~] = PI7_gen_mapeo_actuadas_fun(P, zeros(1,4), tau_articular);

    fprintf('Torque Articular (Hombro q2):  %.1f mN·m\n', tau_articular(2) * 1000);
    fprintf('Torque Actuador  (Motor hombro): %.1f mN·m\n', tau_actuador(2) * 1000);

    % --- GRAFICAR LA POSTURA DE REFERENCIA ---
    %figure('Name', 'Postura de Peor Caso Estático', 'NumberTitle', 'off');
    %R.plot(q_ref);
    %title('Validacion Manual: Brazo extendido (q = [0 0 0 0])');


    