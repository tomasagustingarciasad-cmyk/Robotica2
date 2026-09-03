clc, clear, close all;

% 1. Inicialización del Robot
robot_3gdl;

% 2. Generación de Trayectoria de Prueba
q_inicial = [pi/4, pi/6, -pi/4, 0];
q_final   = [0.7854, 1.1139, -1.3467, 0.7564];
muestras  = 50; 
dt        = 0.1; % Paso de tiempo asumido (s)

% jtraj entrega directamente las derivadas articulares
[Q, Qd, Qdd] = jtraj(q_inicial, q_final, muestras);

% 3. Cálculo de Cinemática Directa (Espacio Cartesiano)
P_cart = zeros(muestras, 3);
for i = 1:muestras
    T_i = R.fkine(Q(i,:));
    P_cart(i,:) = T_i.t'; 
end

% 4. Derivación Numérica Cartesiana
% Se utiliza diff ajustado por el diferencial de tiempo dt para obtener unidades reales
V_cart = diff(P_cart) / dt;   
A_cart = diff(V_cart) / dt;   

% =========================================================================
% FIGURA 1: ESPACIO ARTICULAR
% =========================================================================
figure('Name', 'Cinemática Articular', 'NumberTitle', 'off');

subplot(3,1,1);
plot(Q, 'LineWidth', 1.5);
grid on;
title('Posición articular');
xlabel('Muestras'); ylabel('Ángulo [rad]');
legend('q_1', 'q_2', 'q_3', 'q_4', 'Location', 'best');

subplot(3,1,2);
plot(Qd, 'LineWidth', 1.5);
grid on;
title('Velocidad articular');
xlabel('Muestras'); ylabel('Velocidad [rad/s]');

subplot(3,1,3);
plot(Qdd, 'LineWidth', 1.5);
grid on;
title('Aceleración articular');
xlabel('Muestras'); ylabel('Aceleración [rad/s^2]');

% =========================================================================
% FIGURA 2: ESPACIO CARTESIANO (Efector Final)
% =========================================================================
figure('Name', 'Cinemática Cartesiana', 'NumberTitle', 'off', 'Units', 'normalized', 'Position', [0.5, 0.1, 0.4, 0.8]);

% --- POSICIÓN ---
subplot(3,1,1);
hold on;
% X: Línea gruesa continua
plot(P_cart(:,1), 'LineWidth', 4, 'LineStyle', '-', 'DisplayName', 'X');
% Y: Línea media con guiones
plot(P_cart(:,2), 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Y');
% Z: Línea fina punteada
plot(P_cart(:,3), 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', 'Z');
hold off;
grid on;
title('Posición cartesiana del efector final');
xlabel('Muestras'); ylabel('Coordenadas [m]');
legend('Location', 'best');
xlim([1, muestras]);

% --- VELOCIDAD ---
subplot(3,1,2);
hold on;
plot(V_cart(:,1), 'LineWidth', 4, 'LineStyle', '-', 'DisplayName', 'V_x');
plot(V_cart(:,2), 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'V_y');
plot(V_cart(:,3), 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', 'V_z');
hold off;
grid on;
title('Velocidad cartesiana del efector final');
xlabel('Muestras'); ylabel('Velocidad [m/s]');
legend('Location', 'best');
xlim([1, muestras-1]);

% --- ACELERACIÓN ---
subplot(3,1,3);
hold on;
plot(A_cart(:,1), 'LineWidth', 4, 'LineStyle', '-', 'DisplayName', 'a_x');
plot(A_cart(:,2), 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'a_y');
plot(A_cart(:,3), 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', 'a_z');
hold off;
grid on;
title('Aceleración cartesiana del efector final');
xlabel('Muestras'); ylabel('Aceleración [m/s^2]');
legend('Location', 'best');
xlim([1, muestras-2]);