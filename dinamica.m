% =========================================================================
% ANÁLISIS DINÁMICO COMPLET: INERCIA Y FRICCIÓN (NEWTON-EULER)
% =========================================================================
clc, clear, close all;

% --- 1. PARAMETRIZACIÓN FÍSICA ---
L1 = 0.000;  L2 = 0.150;  L3 = 0.150;  L4 = 0.050; % Longitudes (m)
m1 = 0.0744; m2 = 0.0744; m3 = 0.0248; m4 = 0.0184; % Masas (kg)

% Aproximación de Tensores de Inercia (Cilindros esbeltos sobre el eje X)
% Formula: Iyy = Izz = (1/12) * m * L^2
I_min = 1e-6; % Valor mínimo para evitar singularidades
I2_yy = (1/12) * m2 * L2^2; 
I3_yy = (1/12) * m3 * L3^2;
I4_yy = (1/12) * m4 * L4^2;

I1 = diag([I_min, I_min, I_min]); % Base estática
I2 = diag([I_min, I2_yy, I2_yy]);
I3 = diag([I_min, I3_yy, I3_yy]);
I4 = diag([I_min, I4_yy, I4_yy]);

% Aproximación de Fricción de Coulomb en base a PLA (CoF = 0.6)
% Asumiendo ejes de 4mm de diámetro (r = 0.002m)
% Tc = CoF * Fuerza_Normal * radio
radio_eje = 0.002;
Tc_estimado = 0.6 * ((m2+m3+m4)*9.81) * radio_eje; 
vector_friccion = [Tc_estimado, -Tc_estimado]; % Fricción simétrica [Tc+, Tc-]

% --- 2. DEFINICIÓN DE ESLABONES Y ROBOT ---
% [theta, d, a, alpha]
L_link(1) = Link('d', L1, 'a', 0,  'alpha', pi/2, 'standard');
L_link(2) = Link('d', 0,  'a', L2, 'alpha', 0,    'standard');
L_link(3) = Link('d', 0,  'a', L3, 'alpha', 0,    'standard');
L_link(4) = Link('d', 0,  'a', L4, 'alpha', 0,    'standard');

% Asignación de propiedades dinámicas
L_link(1).m = m1; L_link(1).r = [0, 0, L1/2]; L_link(1).I = I1; L_link(1).Tc = vector_friccion;
L_link(2).m = m2; L_link(2).r = [L2/2, 0, 0]; L_link(2).I = I2; L_link(2).Tc = vector_friccion;
L_link(3).m = m3; L_link(3).r = [L3/2, 0, 0]; L_link(3).I = I3; L_link(3).Tc = vector_friccion;
L_link(4).m = m4; L_link(4).r = [L4/2, 0, 0]; L_link(4).I = I4; L_link(4).Tc = vector_friccion;

R = SerialLink(L_link, 'name', 'Brazo PLA Dinamico');
R.gravity = [0; 0; -9.81];

% --- 3. GENERACIÓN DE TRAYECTORIA Y CÁLCULO DE DINÁMICA INVERSA ---
disp('--- Resolviendo Ecuaciones de Newton-Euler ---');

% Generar una trayectoria de 2 segundos (100 muestras)
t = linspace(0, 2, 100);
q_inicial = [0, pi/4, -pi/4, 0];
q_final   = [0, 0, 0, 0];

% Cálculo de posición (q), velocidad (qd) y aceleración (qdd) articular
[Q, Qd, Qdd] = jtraj(q_inicial, q_final, t);

% Forzar restricción del paralelogramo en toda la trayectoria
Q(:, 4)   = -(Q(:, 2) + Q(:, 3));
Qd(:, 4)  = -(Qd(:, 2) + Qd(:, 3));
Qdd(:, 4) = -(Qdd(:, 2) + Qdd(:, 3));

% Dinámica inversa (Cálculo de Torques)
Tau = R.rne(Q, Qd, Qdd);

% --- 4. VISUALIZACIÓN DE RESULTADOS ---
figure('Name', 'Consumo de Torque Dinamico', 'NumberTitle', 'off');

% Graficar Torque del Motor 2 (Hombro) y Motor 3 (Codo)
plot(t, Tau(:, 2), 'b-', 'LineWidth', 1.5); hold on;
plot(t, Tau(:, 3), 'r--', 'LineWidth', 1.5);
grid on;
title('Torque requerido durante el movimiento (Inercia + Gravedad + Fricción)');
xlabel('Tiempo (segundos)');
ylabel('Torque (N.m)');
legend('Motor 2 (Hombro)', 'Motor 3 (Codo)', 'Location', 'best');