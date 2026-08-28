function [q1, phi, info] = pose_desde_teach(R_deg, P_deg, Y_deg)
%POSE_DESDE_TEACH Convierte la orientacion que MUESTRA el panel Teach en los
%   dos unicos angulos que este brazo puede realmente controlar.
%
%   [Q1, PHI] = POSE_DESDE_TEACH(R, P, Y) toma los tres numeros R, P, Y
%   (en GRADOS) del panel de R.teach y devuelve, en RADIANES:
%       Q1  : giro de la base
%       PHI : cabeceo del efector (phi = q2+q3+q4, positivo = punta arriba)
%
%   Por que no alcanza con leer "R" del panel:
%   el panel usa la convencion XYZ, tr2rpy(T,'xyz'). Esa descomposicion tiene
%   DOS ramas para la misma orientacion: cuando Y sale +90 resulta R = phi y
%   P = q1, pero cuando Y sale -90 los dos aparecen corridos 180 grados. Si
%   uno copiara "R" a ojo, la mitad del espacio de trabajo daria mal. Aca se
%   reconstruye la matriz de rotacion y se extraen los angulos de ella, que
%   es indiferente a la rama.
%
%   Estructura de la rotacion de este brazo:  Rz(q1) * Rx(90) * Rz(phi).
%   Desarrollandola, la tercera fila queda [sin(phi)  cos(phi)  0] y la
%   tercera columna [sin(q1)  -cos(q1)  0]', de donde salen ambos angulos.
%
%   INFO.alcanzable es false si la orientacion pedida NO es de esa forma
%   (el brazo no tiene alabeo de muñeca). INFO.error_deg dice cuanto se
%   aparta; PHI y Q1 devuelven en ese caso la orientacion alcanzable mas
%   cercana.
%
%   Ver tambien IKINE_3GDL, ROBOT_3GDL_CIN_INV.

T = rpy2tr(R_deg, P_deg, Y_deg, 'xyz', 'deg');
M = T(1:3, 1:3);

phi = atan2(M(3,1), M(3,2));
q1  = atan2(M(1,3), -M(2,3));

% Toda orientacion alcanzable tiene el eje z de la herramienta HORIZONTAL,
% es decir M(3,3) = 0. Cuanto se aparta de cero mide lo inalcanzable que es.
err = asin(min(abs(M(3,3)), 1));
info = struct('alcanzable', err < deg2rad(0.5), 'error_deg', rad2deg(err));
end
