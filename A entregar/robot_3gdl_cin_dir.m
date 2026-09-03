clc, clear, close all;

% Ejecutar el script con los parámetros DH
robot_3gdl;
% b. Definición de un vector de posiciones articulares
q = [pi/4, pi/6, -pi/4, 0]; 
% c. Definición de un vector de booleanos para sistemas de referencia
% [sistema{0}, sistema{1}, sistema{2}, sistema{3}]
sistemas = [1, 1, 1, 1, 1];  % 1 = visualizar, 0 = ocultar

% d. Ploteo del robot
figure('Name', 'Sistemas de Referencia DH - 3 GDL', 'NumberTitle', 'off');
R.teach(q, 'workspace', workspace);
hold on;

% e. Bucle para graficar sistemas de referencia
[T, all] = R.fkine(q);

for i = 1:R.n
    if sistemas(i+1) == 1
        if i == 1
            trplot(R.base, 'length', 0.1, 'frame', num2str(i-1), 'color', 'k');
        else
            trplot(all(i-1), 'length', 0.1, 'frame', num2str(i-1), 'color', 'k');
        end
    end
end
% Graficar el sistema de referencia de la herramienta final (Tool Center Point)
trplot(T, 'length', 0.1, 'frame', num2str(R.n), 'color', 'r');

title('Brazo 3 GDL - Sistemas de Referencia');
grid on;
hold off;