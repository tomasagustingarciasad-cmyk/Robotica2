%% 4. FUNCIÓN LOCAL DE CINEMÁTICA INVERSA ANALÍTICA
function [q, alcanzable] = cinematica_analitica(X, Y, Z, Pitch_deg, L1, L2, L3, L4)
    % Altura de la base según tu definición (R.base)
    Z_offset = 0; 
    
    % Convertir Pitch a radianes
    phi = Pitch_deg * (pi / 180);
    
    % 1. Calcular rotación de la base (q1)
    q1 = atan2(Y, X);
    
    % Distancia horizontal (radio en el plano XY)
    r = sqrt(X^2 + Y^2);
    
    % 2. Encontrar coordenadas de la muñeca (AQUÍ RESTAMOS L1)
    rw = r - L4 * cos(phi);
    zw = Z - Z_offset - L1 - L4 * sin(phi);
    
    % 3. Resolver codo (q3) mediante Ley del Coseno
    cos_q3 = (rw^2 + zw^2 - L2^2 - L3^2) / (2 * L2 * L3);
    
    % Comprobar límites físicos (dominio del arcocoseno)
    if cos_q3 > 1 || cos_q3 < -1
        q = [0, 0, 0, 0];
        alcanzable = false;
        return;
    end
    alcanzable = true;
    
    % Seleccionamos postura "codo arriba" (signo negativo)
    q3 = -acos(cos_q3);
    
    % 4. Resolver hombro (q2)
    alpha = atan2(zw, rw);
    beta = atan2(L3 * sin(q3), L2 + L3 * cos(q3));
    q2 = alpha - beta;
    
    % 5. Resolver ángulo final de la herramienta (q4)
    q4 = phi - q2 - q3;
    
    % Salida de la función
    q = [q1, q2, q3, q4];
end