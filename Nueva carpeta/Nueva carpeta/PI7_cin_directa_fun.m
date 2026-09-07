function [T, info] = PI7_cin_directa_fun(P, q)
%PI7_CIN_DIRECTA_FUN  Cinematica directa explicita del brazo PI7, sin RTB.
%
% QUE HACE : evalua el producto de matrices de la DH estandar de los cuatro
%            eslabones, con base y tool, para una o varias posturas. Escrita
%            a mano y sin dependencias para poder contrastarla contra
%            R.fkine: si las dos coinciden, el error no esta en la
%            interpretacion de la tabla DH.
%            No grafica, no imprime, no modifica P.
%
% RECIBE   : P  struct de PI7_gen_parametros_params.
%                 Usa: P.geom.dh, P.geom.offset_deg, P.geom.T_base,
%                      P.geom.T_tool, P.qlim_deg, P.sim.tol_topes_rad.
%            q  N x 4 coordenadas ARTICULARES [rad]. Un vector de 4
%               elementos se toma como una unica muestra.
%               OJO: son q, NO qa. La traduccion desde las coordenadas
%               actuadas la hace PI7_gen_mapeo_actuadas_fun (D10).
%
% DEVUELVE : T     4 x 4 x N, pose del TCP respecto del marco base del mundo
%                  (incluye T_base y T_tool).
%            info  struct:
%                  .p          N x 3, posicion del TCP [m]
%                  .r          N x 1, distancia al eje de la base [m]
%                  .rpy        N x 3, RPY secuencia XYZ [rad] (00_DIRECTIVAS
%                              §3.1). Convencion RTB: R = Rx(r)*Ry(p)*Rz(y)
%                  .rpy_singular  N x 1 logical, cos(pitch) ~ 0: roll y yaw
%                              no estan separados
%                  .phi        N x 1, cabeceo del marco 4 (la canasta)
%                              respecto de la horizontal [rad].
%                              phi = th2+th3+th4. En P3 con el paralelogramo
%                              vale phi = qs (ver nota al pie)
%                  .origenes   N x 3 x 6, origenes de [base, m1, m2, m3, m4,
%                              TCP] en el marco del mundo [m]. Sirve para
%                              graficar y para brazos de palanca
%                  .topes.ok / .viol / .margen_deg / .filas_violadas
%                  .omitidas   cellstr de chequeos no corridos
%
% DEPENDE  : nada. MATLAB base. No requiere Robotics Toolbox.
%
% CONVENCIONES (00_DIRECTIVAS §3.1 y §3.3)
%   DH estandar (Corke), orden de columnas de P.geom.dh: [theta d a alpha].
%   La columna theta queda en CERO: el angulo lo aporta q mas el offset,
%   igual que hace PI7_gen_construir_robot_fun. Si no es cero, se aborta,
%   porque el constructor la ignora y los dos modelos divergirian en
%   silencio.
%   Angulos en radianes en la interfaz; los topes entran en grados desde P.
%
% NOTA SOBRE phi
%   El marco 4 tiene alpha = 0, igual que los marcos 2 y 3: toda la cadena
%   del hombro para adelante es plana. Por eso el cabeceo del ultimo eslabon
%   respecto de la horizontal es la suma th2+th3+th4, y es exactamente la
%   variable que la canasta tiene que mantener en cero durante el transporte.
%   Con la topologia P3 (q4 = -(q2+q3) + qs) resulta phi = qs: el angulo
%   comandado al servo ES el cabeceo de la canasta. Verificado por el chequeo
%   7 de PI7_gen_mapeo_actuadas_test.
%
% AUTOR / FECHA : Equipo PI7 - Robotica II, UNCuyo - 2026-09-03
% -------------------------------------------------------------------------

%% 1. Validacion de P ------------------------------------------------------
if nargin < 2
    error('PI7:cinDirecta:sinArgumentos', ...
        'Uso: [T, info] = PI7_cin_directa_fun(P, q), con q en N x 4 [rad].');
end
campos = {'geom', 'qlim_deg', 'sim'};
for k = 1:numel(campos)
    if ~isstruct(P) || ~isfield(P, campos{k})
        error('PI7:cinDirecta:faltaCampo', ...
            'P no tiene el campo "%s". Revisar PI7_gen_parametros_params.', campos{k});
    end
end
subgeom = {'dh', 'offset_deg', 'T_base', 'T_tool'};
for k = 1:numel(subgeom)
    if ~isfield(P.geom, subgeom{k})
        error('PI7:cinDirecta:faltaCampoGeom', 'P.geom no tiene el campo "%s".', subgeom{k});
    end
end

dh = P.geom.dh;
if ~isequal(size(dh), [4 4])
    error('PI7:cinDirecta:dhTamano', 'P.geom.dh es %dx%d y tiene que ser 4x4.', ...
        size(dh,1), size(dh,2));
end
if any(isnan(dh(:)))
    error('PI7:cinDirecta:dhNaN', 'P.geom.dh contiene NaN: no hay cinematica que evaluar.');
end
if any(dh(:,1) ~= 0)
    error('PI7:cinDirecta:dhTheta', ...
        ['La columna theta de P.geom.dh no es cero. PI7_gen_construir_robot_fun ' ...
         'la ignora (el angulo entra por q + offset), asi que esta CD y R.fkine ' ...
         'darian resultados distintos sin avisar. Mantener theta = 0 y usar ' ...
         'P.geom.offset_deg.']);
end

%% 2. Normalizacion de q ---------------------------------------------------
if isempty(q)
    error('PI7:cinDirecta:sinQ', 'q vacio. Se espera N x 4 en radianes.');
end
if isvector(q) && numel(q) == 4
    q = q(:).';
end
if size(q, 2) ~= 4
    error('PI7:cinDirecta:tamQ', ...
        ['q tiene %d columnas y se esperan 4 (coordenadas ARTICULARES, en ' ...
         'radianes). Si tenes coordenadas actuadas, pasalas primero por ' ...
         'PI7_gen_mapeo_actuadas_fun.'], size(q, 2));
end
if ~isreal(q) || any(~isfinite(q(:)))
    error('PI7:cinDirecta:qNoFinito', 'q contiene NaN, Inf o valores complejos.');
end
N = size(q, 1);

off = deg2rad(P.geom.offset_deg(:)).';        % [rad] 1x4, cero DH - cero mecanico

%% 3. Producto de matrices -------------------------------------------------
T        = zeros(4, 4, N);
origenes = zeros(N, 3, 6);
rpy      = zeros(N, 3);
sing_rpy = false(N, 1);

for k = 1:N
    A = P.geom.T_base;                        % marco del mundo -> marco 0
    origenes(k, :, 1) = A(1:3, 4).';

    for i = 1:4
        th = q(k, i) + off(i);                % [rad] angulo real del eslabon i
        A  = A * matriz_dh(th, dh(i,2), dh(i,3), dh(i,4));
        origenes(k, :, i+1) = A(1:3, 4).';
    end

    Tk = A * P.geom.T_tool;                   % marco 4 -> TCP
    T(:, :, k) = Tk;
    origenes(k, :, 6) = Tk(1:3, 4).';

    [rpy(k, :), sing_rpy(k)] = rot2rpy_xyz(Tk(1:3, 1:3));
end

%% 4. Magnitudes derivadas -------------------------------------------------
info.p            = squeeze(origenes(:, :, 6));
if N == 1, info.p = reshape(info.p, 1, 3); end
info.r            = hypot(info.p(:,1), info.p(:,2));            % [m]
info.rpy          = rpy;                                        % [rad]
info.rpy_singular = sing_rpy;
info.phi          = sum(q(:, 2:4), 2) + sum(off(2:4));          % [rad] cabeceo canasta
info.origenes     = origenes;

%% 5. Topes articulares ----------------------------------------------------
omitidas = {};
if isfield(P.sim, 'tol_topes_rad') && ~isempty(P.sim.tol_topes_rad) ...
        && ~isnan(P.sim.tol_topes_rad)
    tol = P.sim.tol_topes_rad;                % [rad]
else
    tol = 1e-9;                               % [rad] margen de punto flotante, no fisico
    omitidas{end+1} = 'P.sim.tol_topes_rad ausente: se uso 1e-9 rad';
end

qlim    = deg2rad(P.qlim_deg);                % [rad] 4x2
lim_inf = repmat(qlim(:,1).', N, 1);
lim_sup = repmat(qlim(:,2).', N, 1);
viol    = (q < lim_inf - tol) | (q > lim_sup + tol);

info.topes.viol           = viol;
info.topes.ok             = ~any(viol(:));
info.topes.margen_deg     = rad2deg(min(min(q - lim_inf, lim_sup - q), [], 1));
info.topes.filas_violadas = find(any(viol, 2));

omitidas{end+1} = ['topes de las coordenadas ACTUADAS y rango mecanico del ' ...
    'cabeceo: no se verifican aca, van por PI7_gen_mapeo_actuadas_fun'];
info.omitidas = omitidas;
info.tol      = tol;
info.ok       = info.topes.ok;

end
% =========================================================================

function A = matriz_dh(theta, d, a, alpha)
% Transformacion elemental de la DH ESTANDAR (Corke), marco i-1 -> marco i.
ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);
A = [ ct, -st*ca,  st*sa,  a*ct ;
      st,  ct*ca, -ct*sa,  a*st ;
       0,     sa,     ca,     d ;
       0,      0,      0,     1 ];
end

function [rpy, singular] = rot2rpy_xyz(Rm)
% Descomposicion RPY en secuencia XYZ, convencion del RTB:
%   Rm = rotx(roll) * roty(pitch) * rotz(yaw)
% Devuelve [roll pitch yaw] en radianes. singular = true si cos(pitch) ~ 0,
% caso en el que roll y yaw no son separables (se fija roll = 0).
TOL_GIMBAL = 1e-9;                 % [-] margen de punto flotante sobre cos(pitch)

pitch = asin(max(min(Rm(1,3), 1), -1));
cp    = cos(pitch);
if abs(cp) < TOL_GIMBAL
    singular = true;
    roll     = 0;
    yaw      = atan2(Rm(2,1), Rm(2,2));
else
    singular = false;
    roll     = atan2(-Rm(2,3), Rm(3,3));
    yaw      = atan2(-Rm(1,2), Rm(1,1));
end
rpy = [roll, pitch, yaw];
end