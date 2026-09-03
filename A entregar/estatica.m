% --- ANÁLISIS ESTÁTICO DE TORQUES (GRAVEDAD) ---
clc, clear, close all;

% =========================================================================
% 1. ZONA DE PARAMETRIZACIÓN (Modifique estos valores según corresponda)
% =========================================================================

% Parámetros geométricos (Longitudes en metros)
L1 = 0.000; % Altura de la base
L2 = 0.150; % Longitud del brazo principal
L3 = 0.150; % Longitud del antebrazo
L4 = 0.050; % Longitud del efector final

% Parámetros de masa (Masa en kilogramos)
m1 = 0.0744; % Masa del primer eslabón (Base/Soporte)
m2 = 0.0744; % Masa del segundo eslabón (Brazo principal)
m3 = 0.0248; % Masa del tercer eslabón (Antebrazo)
m4 = 0.0184; % Masa del efector final (18.4 gramos)
m5 = 4/3*pi*0.01^3*7800;  % Masa pelotita (32 gramos aprox)

% Aceleración de la gravedad [x, y, z] en m/s^2
gravedad = [0; 0; -9.81]; 

% =========================================================================
% 2. CONSTRUCCIÓN DEL MODELO CINEMÁTICO Y DINÁMICO
% =========================================================================

% Tabla DH Estándar: [theta, d, a, alpha]
L(1) = Link('d', L1, 'a', 0,  'alpha', pi/2, 'standard');
L(2) = Link('d', 0,  'a', L2, 'alpha', 0,    'standard');
L(3) = Link('d', 0,  'a', L3, 'alpha', 0,    'standard');
L(4) = Link('d', 0,  'a', L4, 'alpha', 0,    'standard');

% Asignación de masas
L(1).m = m1;
L(2).m = m2;
L(3).m = m3;
L(4).m = m4+m5;

% Asignación de Centros de Masa (CoM) - Vector [rx, ry, rz]
% Al asumir barras homogéneas, el CoM se sitúa a la mitad del eslabón.
% En la convención DH estándar, la longitud del eslabón transcurre sobre el eje X.
L(1).r = [0, 0, L1/2]; 
L(2).r = [L2/2, 0, 0];
L(3).r = [L3/2, 0, 0];
L(4).r = [L4/2, 0, 0];

% Ensamblaje del robot
R = SerialLink(L, 'name', 'Brazo 4 GDL Estatico');
R.gravity = gravedad;

% Restricciones y configuración base
R.qlim = [
    -180, 180;
     -90,  90;
     -90,  90;
    -180, 180
] * pi/180;

R.offset = [0, 0, 0, 0];
R.base = transl(0, 0, 0.05);

% =========================================================================
% 3. CÁLCULO DE TORQUE EN UNA POSTURA ESPECÍFICA
% =========================================================================

disp('--- CÁLCULO DE TORQUES ESTÁTICOS ---');

% Defina la postura a evaluar (en radianes). 
% Recuerde que q4 debe mantener la restricción del paralelogramo: q4 = -(q2 + q3)
q_eval = [0, 0, 0, 0]; 

% Forzar restricción geométrica para garantizar que el modelo sea fiel al hardware
q_eval(4) = -(q_eval(2) + q_eval(3));

% La función gravload calcula el torque necesario en cada motor para 
% compensar la gravedad en la postura 'q_eval', asumiendo velocidad y aceleración cero.
tau_gravedad = R.gravload(q_eval);

% Mostrar resultados en consola
fprintf('Postura evaluada (Grados): [%.2f, %.2f, %.2f, %.2f]\n', q_eval * 180/pi);
fprintf('Torque requerido en Motor 1 (Base): \t %8.4f N.m\n', tau_gravedad(1));
fprintf('Torque requerido en Motor 2 (Hombro):\t %8.4f N.m\n', tau_gravedad(2));
fprintf('Torque requerido en Motor 3 (Codo): \t %8.4f N.m\n', tau_gravedad(3));
fprintf('Torque requerido en Articulacion 4: \t %8.4f N.m\n', tau_gravedad(4));

% Visualización
figure('Name', 'Analisis Estatico', 'NumberTitle', 'off');
R.plot(q_eval, 'workspace', [-0.5 0.5 -0.5 0.5 0 0.6], 'scale', 0.5);
title('Evaluación Estática de Torques');