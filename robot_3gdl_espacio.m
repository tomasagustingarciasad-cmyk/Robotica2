% --- SCRIPT PARA EL CÁLCULO Y GRAFICACIÓN DEL ESPACIO DE TRABAJO ---
clc, clear, close all;

% 1. Cargar la definición base del robot
robot_3gdl;

% 2. Configuración del método de Monte Carlo
num_muestras = 15000; % Cantidad de puntos aleatorios a evaluar

disp('--- Calculando el espacio de trabajo (Monte Carlo) ---');

% 3. Generar matrices de ángulos aleatorios respetando los límites articulares (R.qlim)
q1_rand = unifrnd(R.qlim(1,1), R.qlim(1,2), num_muestras, 1);
q2_rand = unifrnd(R.qlim(2,1), R.qlim(2,2), num_muestras, 1);
q3_rand = unifrnd(R.qlim(3,1), R.qlim(3,2), num_muestras, 1);

% El cuarto ángulo se calcula analíticamente por la restricción del paralelogramo
q4_rand = -(q2_rand + q3_rand);

% Matriz completa de posturas articulares (N x 4)
Q_montecarlo = [q1_rand, q2_rand, q3_rand, q4_rand];

% Preasignar la matriz de coordenadas (X, Y, Z) para optimizar memoria
puntos_xyz = zeros(num_muestras, 3);

% 4. Bucle para calcular la cinemática directa de cada postura generada
for i = 1:num_muestras
    % Obtener la matriz de transformación homogénea para el i-ésimo vector articular
    T_i = R.fkine(Q_montecarlo(i, :));
    
    % Extraer la posición translacional (X, Y, Z) de la matriz T
    puntos_xyz(i, :) = T_i.t'; 
end

% 5. Graficar la nube de puntos del espacio de trabajo
figure('Name', 'Espacio de Trabajo del Robot 3 GDL', 'NumberTitle', 'off');
plot3(puntos_xyz(:, 1), puntos_xyz(:, 2), puntos_xyz(:, 3), '.', 'Color', [0 0.447 0.741], 'MarkerSize', 2);
hold on;

% Graficar la base del robot como referencia espacial en el origen (ajustada a Z=0.05)
trplot(transl(0, 0, 0.05), 'length', 0.1, 'frame', 'Base', 'color', 'k');

title('Espacio de Trabajo (Nube de Puntos - Monte Carlo)');
xlabel('Eje X (m)');
ylabel('Eje Y (m)');
zlabel('Eje Z (m)');
grid on;
axis equal;
view(3);
hold off;

disp('--- Visualización del espacio de trabajo finalizada ---');