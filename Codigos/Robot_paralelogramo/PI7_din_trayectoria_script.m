%% PI7_din_trayectoria_script.m - Dinámica Inversa por Lagrange
clc; close all; % No usamos 'clear' para heredar el workspace

% Ejecuta tu script cinemático en segundo plano para obtener q_traj, t, etc.
fprintf('Ejecutando cinemática base...\n');
run('PI7_cin_trayectoria_script.m');

% 2. PURGAR NaNs DE LOS ACTUADORES
P.act.usar_actuadores = true;
P.act.Jm_act = [57e-7, 57e-7, 57e-7, 0]; % [kg·m^2] Inercia NEMA 17 y Servo
P.act.G_act  = [1, 1, 1, 1];             % [-] Transmisión directa temporal

% Reconstruir el modelo con los valores purgados
[R, ~] = PI7_gen_construir_robot_fun(P);

% 3. CÁLCULO DINÁMICO: FORMULACIÓN DE LAGRANGE

% 3. CÁLCULO DINÁMICO: FORMULACIÓN DE LAGRANGE
fprintf('\n--- CÁLCULO DINÁMICO (LAGRANGE) ---\n');
tau_articular = zeros(P.sim.n_muestras, R.n);

for i = 1:P.sim.n_muestras
    q   = q_traj(i, :);
    qd  = qd_traj(i, :);
    qdd = qdd_traj(i, :);
    
    % Extracción explícita de matrices
    M = R.inertia(q);
    C = R.coriolis(q, qd);
    G = R.gravload(q);
    
    % Ecuación de Euler-Lagrange
    tau_articular(i, :) = (M * qdd')' + (C * qd')' + G;
end

% 4. MAPEO A TORQUES DE ACTUADORES
[~, ~, tau_actuadores, ~] = PI7_gen_mapeo_actuadas_fun(P, qa_traj, tau_articular, qdda_traj);

% 5. GRÁFICAS DE ESFUERZO DINÁMICO
figure('Name', 'Torques Dinámicos en Actuadores', 'NumberTitle', 'off', 'Units', 'normalized', 'Position', [0.2, 0.2, 0.6, 0.6]);
nombres_motores = {'NEMA 1 (Base)', 'NEMA 2 (Hombro)', 'NEMA 3 (Codo)', 'Servo SG90 (Muñeca)'};
colores = lines(4);

for i = 1:4
    subplot(2, 2, i);
    plot(t, tau_actuadores(:, i) * 1000, 'LineWidth', 2, 'Color', colores(i,:));
    grid on;
    title(['Torque exigido a ', nombres_motores{i}]);
    xlabel('Tiempo [s]'); ylabel('Torque [mN·m]');
    yline(0, 'k--', 'LineWidth', 1);
end