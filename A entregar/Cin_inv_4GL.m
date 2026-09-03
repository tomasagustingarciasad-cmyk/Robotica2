% =========================================================================
%  CINEMATICA INVERSA A PARTIR DE LOS DATOS DEL PANEL "TEACH"
%
%  Flujo de trabajo:
%    1) Corre robot_3gdl_cin y move los sliders hasta una pose que te guste.
%    2) Anota los seis numeros del panel:  x  y  z  R  P  Y
%    3) Pegalos como una fila en la matriz PUNTOS de aca abajo.
%    4) Corre este script: valida cada punto CONTRA LOS TOPES REALES,
%       verifica el recorrido completo y recien ahi lo anima.
%
%  Se usa la cinematica inversa CERRADA (ikine_3gdl), no el solver numerico
%  ikine: el anillo alcanzable de este brazo es muy delgado y el numerico
%  falla o converge a posturas raras con demasiada frecuencia.
% =========================================================================
clc, clear, close all;

robot_3gdl;                    % define R, L1..L4, workspace

% Los topes articulares vienen de robot_3gdl.m (QLIM_DEG), que es la unica
% fuente de verdad. Si hay que corregirlos, se corrigen alla y valen para
% todos los scripts a la vez.

% ------------------------ PUNTOS OBJETIVO --------------------------------
% Una fila por punto, copiada TAL CUAL del panel Teach:
%
%            x        y        z        R        P        Y
%          [m]      [m]      [m]     [deg]    [deg]    [deg]
PUNTOS = [
    0.301   0    0.066    22.5    0    90.0
];
%
%
% Juego de ejemplo que SI cumple los topes, por si queres probar la
% grabacion (descomentalo y comenta el de arriba):
% PUNTOS = [
%      0.108   -0.108    0.333      5.8    -45.0     90.0
%      0.262    0.046    0.231     -0.0     10.0     90.0
% ];

% --------------------------- GRABACION -----------------------------------
% Nombre de archivo para grabar la animacion. Vacio = solo mostrarla.
VIDEO     = '';          % p.ej. 'trayectoria.mp4'
VIDEO_FPS = 25;
PASOS     = 60;          % cuadros por tramo (mas = mas suave y mas largo)
QUIETO    = [12 20];     % cuadros quietos al principio y al final

geom = struct('L2', L2, 'L3', L3, 'L4', L4, 'zb', 0.05, 'qlim', R.qlim);
qActual = [112.5, 90.6, -41.3, -60]*pi/180;      % postura de arranque

% La postura de arranque tambien tiene que respetar los topes.
vArr = (qActual < R.qlim(:,1).') | (qActual > R.qlim(:,2).');
if any(vArr)
    fprintf('\nATENCION: la postura de arranque esta FUERA de topes:\n');
    for a = find(vArr)
        fprintf('   q%d = %.1f deg, fuera de [%.0f, %.0f]\n', a, ...
                rad2deg(qActual(a)), rad2deg(R.qlim(a,1)), rad2deg(R.qlim(a,2)));
    end
    qActual = min(max(qActual, R.qlim(:,1).'), R.qlim(:,2).');
    fprintf('   Se recorta a [%.1f %.1f %.1f %.1f] deg para poder seguir.\n', ...
            rad2deg(qActual));
end

% --------------------------- MODO CONSOLA --------------------------------
if isempty(PUNTOS)
    disp('--- Carga manual (dejar vacio y Enter para terminar) ---');
    while true
        x = input('x [m] : ');
        if isempty(x), break, end
        y = input('y [m] : ');
        z = input('z [m] : ');
        Rd = input('R [deg]: ');
        Pd = input('P [deg]: ');
        Yd = input('Y [deg]: ');
        PUNTOS(end+1, :) = [x y z Rd Pd Yd]; %#ok<AGROW>
    end
    if isempty(PUNTOS), disp('Sin puntos. Fin.'); return, end
end

% -------------------- VALIDACION PUNTO POR PUNTO -------------------------
fprintf('\n=================== VALIDACION DE LOS PUNTOS ===================\n');
fprintf('Topes:  q1 [%.0f,%.0f]   q2 [%.0f,%.0f]   q3 [%.0f,%.0f]   q4 [%.0f,%.0f]  deg\n', ...
        QLIM_DEG.');
fprintf(' #    x[mm]  y[mm]  z[mm]   q1[deg] cabeceo[deg]  estado\n');

Qobj = zeros(0, 4);
validos = [];
for i = 1:size(PUNTOS, 1)
    p = PUNTOS(i, :);
    [q1, phi, ori] = pose_desde_teach(p(4), p(5), p(6));

    fprintf('%2d  %7.1f %6.1f %6.1f  %8.2f %10.2f   ', ...
            i, 1000*p(1), 1000*p(2), 1000*p(3), rad2deg(q1), rad2deg(phi));

    sangria = '';
    if ~ori.alcanzable
        sangria = repmat(' ', 1, 46);
        fprintf('ORIENTACION IMPOSIBLE (se aparta %.1f deg).\n', ori.error_deg);
        fprintf('%sEste brazo no tiene alabeo de muneca: el eje z de la\n', sangria);
        fprintf('%sherramienta siempre queda horizontal. Se usa el cabeceo\n', sangria);
        fprintf('%salcanzable mas cercano.\n', sangria);
    end

    % q1 esta SOBREDETERMINADO: sale tanto de la posicion como de la
    % orientacion. Al copiar el panel redondeado los dos no coinciden por
    % centesimas, y usar el de la orientacion dejaria el punto fuera del
    % plano del brazo. Se manda el de la POSICION, que es exacto, y el de
    % la orientacion se usa solo para elegir la rama del brazo.
    q1pos = atan2(p(2), p(1));
    if abs(angdiff(q1, q1pos)) < pi/2
        q1use = q1pos;                 % rama normal
    else
        q1use = q1pos + pi;            % plegado hacia atras (radio negativo)
    end

    [Q, inf1] = ikine_3gdl(p(1), p(2), p(3), phi, geom, q1use);

    if isempty(Q)
        fprintf('%sDESCARTADO -> %s\n', sangria, inf1.motivo);
        continue
    end

    % Entre codo arriba y codo abajo, la mas parecida a la postura anterior
    if isempty(Qobj), ref = qActual; else, ref = Qobj(end, :); end
    [~, k] = min(sum(abs(angdiff(Q, repmat(ref, size(Q,1), 1))), 2));
    q = Q(k, :);

    % Verificacion: la directa tiene que devolver el punto pedido
    Tv = R.fkine(q);  e = norm(Tv.t(:).' - p(1:3));
    if e > 1e-4
        fprintf('%sERROR de verificacion (%.2f mm)\n', sangria, 1000*e);
        continue
    end

    Qobj(end+1, :) = q; %#ok<AGROW>
    validos(end+1) = i; %#ok<AGROW>
    fprintf('%sOK  q = [%7.2f %7.2f %7.2f %7.2f] deg\n', sangria, rad2deg(q));
end

fprintf('================================================================\n');
fprintf('%d de %d puntos utilizables.\n', size(Qobj,1), size(PUNTOS,1));
if isempty(Qobj)
    fprintf('\n>>> EL RECORRIDO NO ES POSIBLE: no queda ningun punto valido.\n');
    return
end

% ------------- VERIFICACION DE TOPES EN TODO EL RECORRIDO ----------------
% No alcanza con que los PUNTOS esten dentro de topes: hay que revisar el
% camino entero, muestra por muestra.
Qcam = qActual;
for i = 1:size(Qobj, 1)
    Qcam = [Qcam; jtraj(Qcam(end,:), Qobj(i,:), PASOS)]; %#ok<AGROW>
end

fprintf('\n=========== TOPES A LO LARGO DEL RECORRIDO (%d muestras) =========\n', ...
        size(Qcam,1));
fprintf(' art    minimo    maximo   |   tope inf   tope sup    margen\n');
posible = true;
for a = 1:4
    mn = min(Qcam(:,a));  mx = max(Qcam(:,a));
    lo = R.qlim(a,1);     hi = R.qlim(a,2);
    margen = min(mn - lo, hi - mx);
    fprintf(' q%d  %8.2f  %8.2f   |  %8.2f   %8.2f  %8.2f', a, ...
            rad2deg(mn), rad2deg(mx), rad2deg(lo), rad2deg(hi), rad2deg(margen));
    if margen < -1e-9
        fprintf('   <-- SE PASA\n');   posible = false;
    elseif rad2deg(margen) < 5
        fprintf('   <-- al filo\n');
    else
        fprintf('\n');
    end
end
fprintf('================================================================\n');
if ~posible
    fprintf('>>> EL RECORRIDO NO ES POSIBLE: se violan topes en el camino.\n');
    return
end
fprintf('>>> EL RECORRIDO ES POSIBLE: no se toca ningun tope.\n\n');

% --------------------------- ANIMACION -----------------------------------
figure('Name', 'Cinematica inversa desde el panel Teach', 'NumberTitle', 'off', ...
       'Position', [120 80 960 720]);
R.plot(qActual, 'workspace', workspace, 'scale', 0.5);
hold on; grid on

for i = 1:size(Qobj, 1)
    p = PUNTOS(validos(i), 1:3);
    plot3(p(1), p(2), p(3), 'p', 'MarkerSize', 14, ...
          'MarkerFaceColor', [1 0.6 0.1], 'MarkerEdgeColor', 'k');
    text(p(1), p(2), p(3) + 0.02, sprintf(' P%d', validos(i)), 'FontWeight', 'bold');
end
title('Puntos tomados del panel Teach');

% Cuadros quietos en las puntas para que el video no arranque ni termine
% de golpe. No afecta la verificacion de topes, que ya se hizo arriba.
Qcam = [repmat(Qcam(1,:), QUIETO(1), 1); Qcam; repmat(Qcam(end,:), QUIETO(2), 1)];

% Recorrido completo del TCP: sirve para encuadrar y para dibujar el rastro.
XYZ = zeros(size(Qcam,1), 3);
for i = 1:size(Qcam,1)
    Ti = R.fkine(Qcam(i,:));  XYZ(i,:) = Ti.t(:).';
end

% Ejes FIJOS: si no, se reescalan cuadro a cuadro y el video queda saltando.
c = (max(XYZ,[],1) + min(XYZ,[],1))/2;
rad = max(max(XYZ,[],1) - min(XYZ,[],1))/2 + 0.09;
daspect([1 1 1]);
xlim([c(1)-rad, c(1)+rad]);
ylim([c(2)-rad, c(2)+rad]);
zlim([0, max(0.25, c(3)+rad)]);
axis manual
view(-37, 22);

grabando = ~isempty(VIDEO);
if grabando
    vw = VideoWriter(VIDEO, 'MPEG-4');
    vw.FrameRate = VIDEO_FPS;
    open(vw);
    fprintf('Grabando %d cuadros en %s ...\n', size(Qcam,1), VIDEO);
end

hRastro = plot3(nan, nan, nan, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2);
szRef = [];

for i = 1:size(Qcam, 1)
    if ~ishandle(hRastro), break, end          % la cerraron a mano
    R.animate(Qcam(i,:));
    set(hRastro, 'XData', XYZ(1:i,1), 'YData', XYZ(1:i,2), 'ZData', XYZ(1:i,3));
    if grabando
        drawnow
        F = getframe(gcf);
        if isempty(szRef), szRef = size(F.cdata); end
        F.cdata = ajustarCuadro(F.cdata, szRef);
        writeVideo(vw, F);
    else
        drawnow limitrate
        pause(0.015);
    end
end

if grabando
    close(vw);
    fprintf('Video guardado en %s\n', which(VIDEO));
end

% El sistema de referencia final, para contrastar con el panel
Tf = R.fkine(Qobj(end,:));
trplot(Tf.T, 'length', 0.08, 'frame', 'obj', 'color', 'r');

fprintf('Pose final alcanzada:\n');
fprintf('   x=%.3f  y=%.3f  z=%.3f  |  R=%.1f  P=%.1f  Y=%.1f\n', ...
        Tf.t, tr2rpy(Tf.T, 'xyz', 'deg'));
fprintf('   (comparalos con el panel del teach para confirmar)\n');

% -------------------------------------------------------------------------
function c = ajustarCuadro(c, sz)
%AJUSTARCUADRO Recorta o rellena para que todos los cuadros midan igual.
%  getframe puede devolver tamanos que varian en un pixel entre capturas,
%  y VideoWriter rechaza el cuadro si no coincide con el primero.
out = 255*ones(sz(1), sz(2), 3, 'uint8');
h = min(size(c,1), sz(1));
w = min(size(c,2), sz(2));
out(1:h, 1:w, :) = c(1:h, 1:w, :);
c = out;
end
