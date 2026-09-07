function [Q, info] = PI7_cin_inversa_cerrada_fun(P, pose, verificar)
%PI7_CIN_INVERSA_CERRADA_FUN  Cinematica inversa en forma cerrada del PI7.
%
% QUE HACE : resuelve analiticamente las cuatro ramas de postura que llevan
%            el TCP a una pose (x, y, z, phi) dada, sin iteracion y sin
%            solver numerico. Devuelve TODAS las ramas y, para las que no
%            sirven, el motivo del descarte. No elige entre ellas: eso es
%            trabajo de PI7_cin_seleccion_postura_fun.
%            No grafica, no imprime, no modifica P.
%
% RECIBE   : P     struct de PI7_gen_parametros_params.
%            pose  N x 4 = [x y z phi].  x,y,z en [m] respecto del marco del
%                  mundo; phi en [rad] es el cabeceo de la canasta respecto
%                  de la horizontal, positivo hacia arriba (retiene).
%                  Un vector de 4 elementos se toma como una unica muestra.
%            verificar  (opcional, logical, default true) realimenta cada
%                  rama valida a PI7_cin_directa_fun y reporta el error de
%                  posicion. Criterio de 00_DIRECTIVAS §7.2: < P.sim.tol_pos.
%
% DEVUELVE : Q     NRAM x 4 x N, coordenadas ARTICULARES [rad].
%                  Q(r,:,k) = postura de la rama r para la muestra k.
%                  NaN en las ramas descartadas.
%            info  struct:
%                  .ramas        1 x NRAM cellstr, nombre de cada rama
%                  .valida       NRAM x N logical
%                  .motivo       NRAM x N cellstr, '' si la rama es valida
%                  .n_validas    N x 1
%                  .sin_solucion N x 1 logical, ninguna rama valida
%                  .singular     N x 1 logical
%                  .singular_motivo  N x 1 cellstr
%                  .err_pos      NRAM x N [m], NaN si no se verifico
%                  .err_pos_max  peor error entre las ramas validas [m]
%                  .omitidas     cellstr de chequeos no corridos
%
% DEPENDE  : PI7_cin_directa_fun (solo si verificar = true). Nada mas.
%            No requiere Robotics Toolbox.
%
% POR QUE (x, y, z, phi) Y NO UNA POSE SE(3) COMPLETA
%   El brazo tiene 4 GDL. Los eslabones 2, 3 y 4 tienen alpha = 0: son una
%   cadena PLANA que vive en el plano vertical que q1 orienta. La unica
%   libertad de orientacion que existe es el cabeceo dentro de ese plano.
%   Pedir una matriz SE(3) generica seria pedir 6 restricciones a 4 GDL.
%   phi = th2 + th3 + th4 es la unica componente de orientacion realizable,
%   y ademas es la fisicamente relevante: es la inclinacion de la canasta.
%
% RAMAS
%   1  base directa  / codo q3 > 0
%   2  base directa  / codo q3 < 0
%   3  base volteada / codo q3 > 0
%   4  base volteada / codo q3 < 0
%   "Base volteada" es q1 girado ~180 grados alcanzando el punto por detras.
%   Casi siempre cae por topes; se calcula igual para no ocultarla.
%
% FRONTERA DE COORDENADAS (04_CODIGO §4)
%   La salida esta en coordenadas ARTICULARES q. Para comandarla hay que
%   pasarla por PI7_gen_mapeo_actuadas_fun, que ademas verifica los topes de
%   los actuadores y el rango mecanico del cabeceo. Esta funcion solo
%   verifica P.qlim_deg. Una rama marcada valida aca puede ser
%   incomandable en P3 (D10).
%
% AUTOR / FECHA : Equipo PI7 - Robotica II, UNCuyo - 2026-09-03
% -------------------------------------------------------------------------

NRAM       = 4;
signo_base = [+1, +1, -1, -1];
signo_codo = [+1, -1, +1, -1];
ramas      = {'base directa / codo q3>0', 'base directa / codo q3<0', ...
              'base volteada / codo q3>0', 'base volteada / codo q3<0'};

%% 1. Validacion de P y extraccion de la geometria -------------------------
if nargin < 2
    error('PI7:cinInversa:sinArgumentos', ...
        'Uso: [Q, info] = PI7_cin_inversa_cerrada_fun(P, pose).');
end
if nargin < 3 || isempty(verificar), verificar = true; end
if ~islogical(verificar) || ~isscalar(verificar)
    error('PI7:cinInversa:verificar', 'El argumento verificar tiene que ser logical escalar.');
end
campos = {'geom', 'qlim_deg', 'sim'};
for k = 1:numel(campos)
    if ~isstruct(P) || ~isfield(P, campos{k})
        error('PI7:cinInversa:faltaCampo', 'P no tiene el campo "%s".', campos{k});
    end
end

dh = P.geom.dh;
if ~isequal(size(dh), [4 4]) || any(isnan(dh(:)))
    error('PI7:cinInversa:dh', 'P.geom.dh tiene que ser 4x4 sin NaN.');
end

% --- La forma cerrada vale para ESTA estructura, no para cualquier DH -----
TOL_ESTRUCTURA = 1e-12;      % [-] margen de punto flotante, no parametro fisico
if any(dh(:,1) ~= 0)
    error('PI7:cinInversa:dhTheta', ...
        'La columna theta de P.geom.dh no es cero. Ver PI7_cin_directa_fun.');
end
if abs(dh(1,3)) > TOL_ESTRUCTURA || abs(dh(1,4) - pi/2) > TOL_ESTRUCTURA
    error('PI7:cinInversa:estructura1', ...
        ['La solucion cerrada supone a1 = 0 y alpha1 = pi/2 (eje de la base ' ...
         'vertical, hombro perpendicular). P.geom.dh fila 1 no cumple.']);
end
if any(abs(dh(2:4,4)) > TOL_ESTRUCTURA) || any(abs(dh(2:4,2)) > TOL_ESTRUCTURA)
    error('PI7:cinInversa:estructura234', ...
        ['La solucion cerrada supone alpha = 0 y d = 0 en los eslabones 2, 3 ' ...
         'y 4 (cadena plana). P.geom.dh no cumple: si el CAD introduce un ' ...
         'desplazamiento fuera del plano hay que rederivar la CI.']);
end

d1 = dh(1,2);   a2 = dh(2,3);   a3 = dh(3,3);   a4 = dh(4,3);   % [m]
if a2 <= 0 || a3 <= 0
    error('PI7:cinInversa:eslabonNulo', ...
        'a2 y a3 tienen que ser positivos. Valen %.4g y %.4g m.', a2, a3);
end

% --- Base: solo traslacion sobre z ---------------------------------------
Tb = P.geom.T_base;
if norm(Tb(1:3,1:3) - eye(3), 'fro') > TOL_ESTRUCTURA || any(abs(Tb(1:2,4)) > TOL_ESTRUCTURA)
    error('PI7:cinInversa:baseGenerica', ...
        ['P.geom.T_base tiene rotacion o traslacion horizontal. La forma ' ...
         'cerrada supone base = transl(0,0,zb). Transforma la pose objetivo ' ...
         'al marco de la base antes de llamar.']);
end
zb = Tb(3,4);                                                   % [m]

% --- Tool: solo traslacion, expresada en el marco 4 ----------------------
Tt = P.geom.T_tool;
if norm(Tt(1:3,1:3) - eye(3), 'fro') > TOL_ESTRUCTURA
    error('PI7:cinInversa:toolRotada', ...
        ['P.geom.T_tool tiene rotacion. La forma cerrada admite tool de ' ...
         'traslacion pura; con rotacion, phi deja de ser el cabeceo del TCP.']);
end
tx = Tt(1,4);  ty = Tt(2,4);  tz = Tt(3,4);                     % [m]

off = deg2rad(P.geom.offset_deg(:)).';                          % [rad] 1x4

%% 2. Normalizacion de pose ------------------------------------------------
if isempty(pose)
    error('PI7:cinInversa:sinPose', 'pose vacia. Se espera N x 4 = [x y z phi].');
end
if isvector(pose) && numel(pose) == 4
    pose = pose(:).';
end
if size(pose, 2) ~= 4
    error('PI7:cinInversa:tamPose', ...
        'pose tiene %d columnas y se esperan 4: [x y z phi].', size(pose, 2));
end
if ~isreal(pose) || any(~isfinite(pose(:)))
    error('PI7:cinInversa:poseNoFinita', 'pose contiene NaN, Inf o complejos.');
end
N = size(pose, 1);

%% 3. Tolerancias ----------------------------------------------------------
omitidas = {};
if isfield(P.sim, 'tol_pos') && ~isempty(P.sim.tol_pos) && ~isnan(P.sim.tol_pos)
    tol_pos = P.sim.tol_pos;                  % [m]
else
    tol_pos = 1e-4;                           % [m] criterio 00_DIRECTIVAS §7.2
    omitidas{end+1} = 'P.sim.tol_pos ausente: se uso 1e-4 m';
end
if isfield(P.sim, 'tol_topes_rad') && ~isempty(P.sim.tol_topes_rad) ...
        && ~isnan(P.sim.tol_topes_rad)
    tol_top = P.sim.tol_topes_rad;            % [rad]
else
    tol_top = 1e-9;                           % [rad] margen de punto flotante
    omitidas{end+1} = 'P.sim.tol_topes_rad ausente: se uso 1e-9 rad';
end
omitidas{end+1} = ['topes de las coordenadas ACTUADAS y rango mecanico del ' ...
    'cabeceo: van por PI7_gen_mapeo_actuadas_fun, no se verifican aca'];

qlim = deg2rad(P.qlim_deg);                   % [rad] 4x2

%% 4. Resolucion -----------------------------------------------------------
Q               = nan(NRAM, 4, N);
valida          = false(NRAM, N);
motivo          = repmat({''}, NRAM, N);
singular        = false(N, 1);
singular_motivo = repmat({''}, N, 1);
err_pos         = nan(NRAM, N);

for k = 1:N
    x = pose(k,1);  y = pose(k,2);  z = pose(k,3);  phi = pose(k,4);

    rho = hypot(x, y);                        % [m] distancia al eje de la base
    yp  = z - zb - d1;                        % [m] altura sobre el eje del hombro

    % El tool puede tener componente tz FUERA del plano del brazo: entonces
    % el TCP no pasa por el eje de la base y rho tiene un piso.
    if rho^2 < tz^2 - tol_pos^2
        motivo(:,k) = {sprintf(['fuera del plano alcanzable: rho = %.1f mm < |tz| ' ...
            '= %.1f mm del tool'], rho*1e3, abs(tz)*1e3)};
        continue
    end
    xp_mag = sqrt(max(rho^2 - tz^2, 0));      % [m] |coordenada radial en el plano|

    if rho < tol_pos && abs(tz) < tol_pos
        singular(k)        = true;
        singular_motivo{k} = ['TCP sobre el eje de la base: q1 indeterminado, ' ...
                              'se devuelve q1 = 0'];
        psi = 0;
    else
        psi = atan2(y, x);                    % [rad]
    end

    for r = 1:NRAM
        xp  = signo_base(r) * xp_mag;         % [m] radial con signo segun la rama
        th1 = psi + atan2(tz, xp);            % [rad]

        % Muñeca = origen del marco 3 = eje de cabeceo q4.
        % Se descuenta el ultimo eslabon mas el tool, ambos girados phi.
        xw = xp - (a4 + tx)*cos(phi) + ty*sin(phi);     % [m]
        yw = yp - (a4 + tx)*sin(phi) - ty*cos(phi);     % [m]
        Rw = hypot(xw, yw);                             % [m]

        if Rw > a2 + a3 + tol_pos
            motivo{r,k} = sprintf(['fuera de alcance: muñeca a %.1f mm del hombro, ' ...
                'maximo %.1f mm'], Rw*1e3, (a2+a3)*1e3);
            continue
        end
        if Rw < abs(a2 - a3) - tol_pos
            motivo{r,k} = sprintf(['dentro del hueco interior: muñeca a %.1f mm, ' ...
                'minimo %.1f mm'], Rw*1e3, abs(a2-a3)*1e3);
            continue
        end
        if Rw < tol_pos
            singular(k) = true;
            singular_motivo{k} = ['muñeca sobre el eje del hombro (a2 = a3): ' ...
                                  'q2 indeterminado'];
        end

        D   = (Rw^2 - a2^2 - a3^2) / (2*a2*a3);
        D   = max(min(D, 1), -1);                       % [-] recorte numerico
        th3 = signo_codo(r) * acos(D);                  % [rad]
        th2 = atan2(yw, xw) - atan2(a3*sin(th3), a2 + a3*cos(th3));   % [rad]
        th4 = phi - th2 - th3;                          % [rad]

        % Angulo real del eslabon -> coordenada articular (00_DIRECTIVAS §3.2)
        qk = envolver_pi([th1, th2, th3, th4] - off);   % [rad]

        % --- Topes ARTICULARES (los del SerialLink) ---
        fuera = (qk < qlim(:,1).' - tol_top) | (qk > qlim(:,2).' + tol_top);
        if any(fuera)
            idx = find(fuera);
            partes = cell(1, numel(idx));
            for j = 1:numel(idx)
                i = idx(j);
                partes{j} = sprintf('q%d = %.1f fuera de [%.1f %.1f]', i, ...
                    rad2deg(qk(i)), P.qlim_deg(i,1), P.qlim_deg(i,2));
            end
            motivo{r,k} = ['tope articular: ' strjoin(partes, '; ') ' [deg]'];
            continue
        end

        Q(r,:,k)    = qk;
        valida(r,k) = true;
    end
end

%% 5. Realimentacion a la CD (00_DIRECTIVAS §7.2) --------------------------
if verificar && any(valida(:))
    for k = 1:N
        for r = 1:NRAM
            if ~valida(r,k), continue, end
            [~, iCD]     = PI7_cin_directa_fun(P, Q(r,:,k));
            err_pos(r,k) = norm(iCD.p - pose(k,1:3));   % [m]
        end
    end
else
    if ~verificar
        omitidas{end+1} = 'verificar = false: no se realimento la CI a la CD';
    end
end

%% 6. Info -----------------------------------------------------------------
info.ramas           = ramas;
info.valida          = valida;
info.motivo          = motivo;
info.n_validas       = sum(valida, 1).';
info.sin_solucion    = info.n_validas == 0;
info.singular        = singular;
info.singular_motivo = singular_motivo;
info.err_pos         = err_pos;
info.err_pos_max     = max(err_pos(valida), [], 'omitnan');
if isempty(info.err_pos_max), info.err_pos_max = NaN; end
info.tol_pos         = tol_pos;
info.tol_topes_rad   = tol_top;
info.omitidas        = omitidas;
info.ok              = ~any(info.sin_solucion) && ...
                       (isnan(info.err_pos_max) || info.err_pos_max < tol_pos);

end
% =========================================================================

function a = envolver_pi(a)
% Lleva angulos al intervalo (-pi, pi]. Es la representacion unica de la
% posicion FISICA de una junta rotativa, que es sobre la que se verifican los
% topes. No usa wrapToPi: eso vive en el Mapping Toolbox.
a = a - 2*pi*floor((a + pi)/(2*pi));
end