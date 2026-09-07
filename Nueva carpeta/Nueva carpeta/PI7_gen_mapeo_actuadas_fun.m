function [q, dq_dqa, tau_a, info] = PI7_gen_mapeo_actuadas_fun(P, qa, tau, qdda)
% PI7_GEN_MAPEO_ACTUADAS_FUN
%
% QUE HACE
%   Unico lugar del proyecto donde vive la topologia del mecanismo (D10, D13).
%   Traduce coordenadas ACTUADAS qa (lo que se le comanda a cada motor) a
%   coordenadas ARTICULARES q (los angulos que entran al SerialLink),
%   devuelve el jacobiano constante del mapeo, convierte pares articulares en
%   pares de actuador, y aporta la inercia de rotor reflejada en coordenadas
%   actuadas (D14).
%   Verifica los topes sobre qa, no sobre q, y verifica por separado la
%   restriccion DERIVADA de rango mecanico del cabeceo total q4.
%   No grafica, no imprime, no modifica P.
%
% POR QUE EXISTE ESTA FUNCION
%   En un robot serie clasico hay un motor sentado en cada articulacion y
%   qa = q: la distincion es vacia. En el PI7 no: los tres NEMA 17 estan en
%   la base (D13) y el antebrazo lo mueve un paralelogramo (lazo L1) cuya
%   manivela pivotea en el eje del hombro. Ese motor impone el angulo del
%   antebrazo respecto de la BASE, no respecto del brazo. Un motor mueve dos
%   angulos articulares. Sin este traductor:
%     - no se puede comandar el resultado de la cinematica inversa;
%     - el torque que devuelve rne no es el par de ningun motor real.
%
% DOS TOPOLOGIAS, DOS FILAS DISTINTAS
%   P.geom.topologia_q4    gobierna la fila de q4 (muñeca, D10)
%   P.geom.topologia_codo  gobierna la fila de q3 (accionamiento, D13)
%   Son independientes. Ninguna tiene valor por defecto: si falta alguna, la
%   funcion falla con error. Que la del codo se eligiera por omision al
%   escribir la formula es exactamente el error que corrigio D13.
%
%   Relaciones implementadas (angulos, sin offsets):
%     codo 'junta'    : q3 = qa3
%     codo 'L1_base'  : q3 = qa3 - qa2         (qa3 = angulo ABSOLUTO)
%     muñeca 'P1'     : q4 = -(q2 + q3)
%     muñeca 'P2'     : q4 = qa4               (servo directo)
%     muñeca 'P3'     : q4 = -(q2 + q3) + qs
%   Con P3 + L1_base queda q4 = -qa3 + qs: el cabeceo depende de UN motor de
%   brazo, no de dos. El hombro deja de intervenir.
%
% MAPEO AFIN, NO LINEAL
%   q = (qa + offset_act) * dq_dqa'
%   El paralelogramo iguala el angulo del balancin al de la manivela MAS una
%   constante de armado, y el cero de cada motor lo fija el homing. Ese
%   termino vive en P.geom.offset_act_deg, en coordenadas ACTUADAS. Es
%   distinto de P.geom.offset_deg, que va del cero articular al cero DH.
%   Los topes se verifican sobre qa TAL COMO SE COMANDA, o sea antes del
%   offset: P.qlim_act_deg esta escrito en unidades de comando.
%
% QUE RECIBE
%   P     struct de PI7_gen_parametros_params. Usa:
%           P.geom.topologia_q4     'P1' | 'P2' | 'P3'
%           P.geom.topologia_codo   'junta' | 'L1_base'
%           P.geom.offset_act_deg   1 x na, offset del cero de cada actuador
%           P.geom.q4_mec_deg       [min max] rango mecanico del cabeceo total
%           P.qlim_act_deg          na x 2, topes de las coordenadas actuadas
%           P.act.usar_actuadores, P.act.Jm_act, P.act.G_act   (D14)
%           P.sim.tol_topes_rad     tolerancia de comparacion (opcional)
%   qa    N x na, coordenadas actuadas en RADIANES. Un vector de na elementos
%         se interpreta como una unica muestra.
%   tau   opcional. N x 4 (o 1 x 4), pares en coordenadas ARTICULARES [N*m],
%         tal cual los devuelve rne / gravload del RTB.
%   qdda  opcional. N x na (o 1 x na), aceleraciones ACTUADAS [rad/s^2]. Si
%         se pasa junto con tau, tau_a incluye el termino de inercia de rotor
%         reflejada. Sin qdda, tau_a es solo el dual del mapeo (exacto para
%         la ESTATICA, incompleto para la dinamica).
%
% QUE DEVUELVE
%   q        N x 4, coordenadas articulares [rad], listas para el SerialLink.
%   dq_dqa   4 x na, jacobiano constante del mapeo.
%   tau_a    N x na, pares en el eje de cada actuador [N*m]. [] si no hay tau.
%   info     struct con:
%              .topologia_q4, .topologia_codo, .na, .nombres
%              .dq_dqa, .dqa_dq            matrices directa e inversa
%              .offset_act_rad             offset aplicado
%              .M_rotor                    na x na, diag(Jm_act .* G_act.^2).
%                                          NaN si falta el datasheet: es un
%                                          dato ausente, no un cero
%              .rotor_incluido             logical
%              .rotor_contamina            logical, true si algun NaN de
%                                          M_rotor multiplico una aceleracion
%                                          no nula y se propago a tau_a
%              .tol                        tolerancia usada [rad]
%              .topes.ok/.viol/.margen_deg/.filas_violadas/.nombres
%              .cabeceo.aplica/.verificado/.ok/.q4_deg/.margen_deg/.motivo
%              .ok                         todo lo efectivamente corrido pasa
%              .omitidas                   cellstr de chequeos no corridos
%
%   ATENCION: info.ok = true con info.omitidas no vacio significa "no se
%   detecto violacion en lo que se pudo verificar", no "esta verificado".
%
% INERCIA DE ROTOR (D14)
%   El RTB asocia L.Jm y L.G una-a-una con cada articulacion. Con actuacion
%   acoplada eso no existe: la reduccion del motor del codo se refiere a su
%   propia manivela, no a q3. Por eso el SerialLink se arma con Jm = 0, G = 1
%   y el termino de rotor entra ACA, en coordenadas actuadas:
%       M_a = dq_dqa' * M * dq_dqa + M_rotor,   M_rotor = diag(Jm_act.*G_act.^2)
%       tau_a = dq_dqa' * tau + M_rotor * qdda_a
%   La ESTATICA no se ve afectada: M_rotor entra solo multiplicando qdda.
%
%   NaN Y ACELERACION NULA (D16). El termino se evalua elemento por elemento
%   y solo donde la aceleracion actuada es DISTINTA de cero. Con qdda_i = 0
%   exacto, la contribucion del rotor i vale cero para cualquier Jm y G
%   FINITOS, asi que un NaN que solo significa "todavia no elegimos el motor"
%   no tiene por que contaminar el resultado. Esto NO es sustituir NaN por
%   cero: donde la aceleracion no es nula el NaN se propaga tal cual, se
%   marca en info.rotor_contamina y se reporta en info.omitidas.
%   Sin este cuidado, el producto matricial qdda * M_rotor devuelve NaN aun
%   con qdda = 0 (en IEEE-754, NaN*0 = NaN) y se recrea el bucle imposible
%   que documenta D09: no se puede obtener el par de motor de la estatica
%   sin haber elegido antes el motor.
%
% DE QUE DEPENDE
%   Nada. No requiere el Robotics Toolbox.
%
% AUTOR / FECHA
%   Equipo PI7 - Robotica II, UNCuyo - 2026-09-03 (rev. D13, D14, D15, D16)
% -------------------------------------------------------------------------

% ---------- 1. Topologia de la muñeca ------------------------------------
if ~isstruct(P) || ~isfield(P, 'geom') || ~isfield(P.geom, 'topologia_q4')
    error('PI7:mapeo:campoAusente', ...
        ['Falta el campo P.geom.topologia_q4. La topologia del cuarto GDL ' ...
         'es una decision de diseño (D10) y sin ella no hay mapeo posible. ' ...
         'Valores admitidos: ''P1'', ''P2'', ''P3''.']);
end
topo = P.geom.topologia_q4;
if isstring(topo); topo = char(topo); end
if ~ischar(topo) || isempty(strtrim(topo))
    error('PI7:mapeo:topologiaVacia', ...
        ['P.geom.topologia_q4 esta vacio: la topologia del cuarto GDL no ' ...
         'esta decidida. Ver 03_DECISIONES D10 y 01_ESPECIFICACION §3.5.']);
end
topo = upper(strtrim(topo));
if ~any(strcmp(topo, {'P1','P2','P3'}))
    error('PI7:mapeo:topologiaInvalida', ...
        ['P.geom.topologia_q4 = ''%s'' no es una topologia reconocida. ' ...
         'Admitidas: ''P1'', ''P2'', ''P3''.'], topo);
end

% ---------- 2. Topologia del accionamiento del codo (D13) ----------------
if ~isfield(P.geom, 'topologia_codo')
    error('PI7:mapeo:codoAusente', ...
        ['Falta el campo P.geom.topologia_codo. Donde va montado el motor ' ...
         'del codo cambia la fila de q3 de la matriz de mapeo y con ella el ' ...
         'par que se le pide al motor del hombro. Es una decision de diseño ' ...
         '(D13, cierra P11). Valores admitidos: ''junta'', ''L1_base''.']);
end
codo = P.geom.topologia_codo;
if isstring(codo); codo = char(codo); end
if ~ischar(codo) || isempty(strtrim(codo))
    error('PI7:mapeo:codoVacio', ...
        ['P.geom.topologia_codo esta vacio: no esta decidido donde va el ' ...
         'motor del codo. Ver 03_DECISIONES D13 y 01_ESPECIFICACION §3.6. ' ...
         'Cargar ''junta'' o ''L1_base'' en PI7_gen_parametros_params.']);
end
codo = upper(strtrim(codo));
if ~any(strcmp(codo, {'JUNTA','L1_BASE'}))
    error('PI7:mapeo:codoInvalido', ...
        ['P.geom.topologia_codo = ''%s'' no es reconocida. ' ...
         'Admitidas: ''junta'', ''L1_base''.'], codo);
end

% ---------- 3. Construccion de la matriz de mapeo ------------------------
% Filas de dq_dqa: una por coordenada articular q1..q4.
% Columnas: una por coordenada actuada.
%
%   q1 = qa1                                       (siempre)
%   q2 = qa2                                       (siempre)
%   q3 = qa3            (codo en la junta)
%   q3 = qa3 - qa2      (codo por L1 desde la base)
%   q4 segun la muñeca, expresado en funcion de q2 y q3 ya resueltos.

switch topo
    case 'P1', na = 3;  nom4 = {};
    case 'P2', na = 4;  nom4 = {'q4'};
    case 'P3', na = 4;  nom4 = {'qs'};
end

f1 = [1, zeros(1, na-1)];                    % fila de q1
f2 = [0, 1, zeros(1, na-2)];                 % fila de q2
switch codo
    case 'JUNTA'
        f3 = [0, 0, 1, zeros(1, na-3)];      % q3 = qa3
        nom3 = 'q3';
    case 'L1_BASE'
        f3 = [0, -1, 1, zeros(1, na-3)];     % q3 = qa3 - qa2
        nom3 = 'q3abs';
end
switch topo
    case 'P1'
        f4 = -(f2 + f3);                     % q4 = -(q2+q3)
    case 'P2'
        f4 = [0 0 0 1];                      % q4 = qa4, servo directo
    case 'P3'
        f4 = -(f2 + f3) + [0 0 0 1];         % q4 = -(q2+q3) + qs
end

dq_dqa  = [f1; f2; f3; f4];                  % 4 x na
nombres = [{'q1', 'q2', nom3}, nom4];

% Inversa. En P1 el mapeo es 3 -> 4 y no es invertible: dqa_dq es la
% pseudo-inversa exacta sobre la variedad alcanzable (deja de lado q4, que es
% dependiente). En P2 y P3 es la inversa verdadera.
if na == 4
    dqa_dq = inv(dq_dqa);
else
    dqa_dq = dq_dqa(1:3, :) \ eye(3);
    dqa_dq = [dqa_dq, zeros(3,1)];           % qa no depende de q4 (dependiente)
end

% ---------- 4. Normalizacion de qa ---------------------------------------
if nargin < 2 || isempty(qa)
    error('PI7:mapeo:sinQa', 'Falta el argumento qa (N x %d, en radianes).', na);
end
if isvector(qa) && numel(qa) == na
    qa = qa(:).';
end
if size(qa, 2) ~= na
    error('PI7:mapeo:tamQa', ...
        ['qa tiene %d columnas y la topologia %s/%s actua %d coordenadas ' ...
         '(%s). qa se espera N x %d, en radianes.'], ...
        size(qa, 2), topo, codo, na, strjoin(nombres, ', '), na);
end
if ~isreal(qa) || any(~isfinite(qa(:)))
    error('PI7:mapeo:qaNoFinito', 'qa contiene NaN, Inf o valores complejos.');
end
N = size(qa, 1);

% ---------- 5. Offset de los actuadores (mapeo afin) ---------------------
omitidas = {};
if isfield(P.geom, 'offset_act_deg') && ~isempty(P.geom.offset_act_deg)
    off_act = P.geom.offset_act_deg(:).';
    if numel(off_act) ~= na
        error('PI7:mapeo:tamOffsetAct', ...
            ['P.geom.offset_act_deg tiene %d elementos y la topologia %s/%s ' ...
             'actua %d coordenadas.'], numel(off_act), topo, codo, na);
    end
    if any(isnan(off_act))
        error('PI7:mapeo:offsetActNaN', ...
            'P.geom.offset_act_deg contiene NaN. Usar 0 si todavia no se midio.');
    end
else
    off_act = zeros(1, na);
    omitidas{end+1} = ['P.geom.offset_act_deg ausente: se supuso cero. El ' ...
        'cero de cada motor no esta definido (P10)'];
end
off_act_rad = off_act * pi / 180;            % [rad]

% ---------- 6. Mapeo directo ---------------------------------------------
q = (qa + off_act_rad) * dq_dqa.';           % N x 4, articulares [rad]

% ---------- 7. Tolerancia de comparacion ---------------------------------
if isfield(P, 'sim') && isfield(P.sim, 'tol_topes_rad') ...
        && ~isempty(P.sim.tol_topes_rad) && ~isnan(P.sim.tol_topes_rad)
    tol = P.sim.tol_topes_rad;
else
    tol = 1e-9;                              % [rad] fallback de punto flotante
    omitidas{end+1} = 'P.sim.tol_topes_rad ausente: se uso 1e-9 rad';
end

% ---------- 8. Topes sobre las coordenadas ACTUADAS ----------------------
if ~isfield(P, 'qlim_act_deg') || isempty(P.qlim_act_deg)
    error('PI7:mapeo:sinTopesActuados', ...
        ['Falta P.qlim_act_deg (%d x 2, en grados). Son los topes FISICOS: ' ...
         'recorrido de cada manivela y del servo. Desde D15 son la fuente de ' ...
         'verdad, y P.qlim_deg se deriva de ellos.'], na);
end
if ~isequal(size(P.qlim_act_deg), [na 2])
    error('PI7:mapeo:tamTopesActuados', ...
        'P.qlim_act_deg es %dx%d y para la topologia %s/%s tiene que ser %dx2.', ...
        size(P.qlim_act_deg,1), size(P.qlim_act_deg,2), topo, codo, na);
end

qlim_act = P.qlim_act_deg * pi / 180;        % na x 2 [rad]
lim_inf  = repmat(qlim_act(:, 1).', N, 1);
lim_sup  = repmat(qlim_act(:, 2).', N, 1);
viol     = (qa < lim_inf - tol) | (qa > lim_sup + tol);

info.topes.viol           = viol;
info.topes.ok             = ~any(viol(:));
info.topes.margen_deg     = min(min(qa - lim_inf, lim_sup - qa), [], 1) * 180/pi;
info.topes.filas_violadas = find(any(viol, 2));
info.topes.nombres        = nombres;

% ---------- 9. Restriccion derivada: rango mecanico del cabeceo ---------
% En P1 y P3 el limite de q4 NO es un tope de actuador: es la interferencia
% del pivote entre antebrazo y soporte de canasta. Se verifica aparte.
info.cabeceo.aplica     = ~strcmp(topo, 'P2');
info.cabeceo.verificado = false;
info.cabeceo.ok         = true;
info.cabeceo.q4_deg     = q(:, 4) * 180 / pi;
info.cabeceo.margen_deg = NaN;
info.cabeceo.motivo     = '';
info.cabeceo.filas_violadas = [];

if info.cabeceo.aplica
    if ~isfield(P.geom, 'q4_mec_deg') || isempty(P.geom.q4_mec_deg) ...
            || any(isnan(P.geom.q4_mec_deg))
        info.cabeceo.motivo = ['P.geom.q4_mec_deg sin definir: no se ' ...
            'verifico el rango mecanico del cabeceo total'];
        omitidas{end+1} = info.cabeceo.motivo;
    else
        q4lim = P.geom.q4_mec_deg(:).' * pi / 180;
        viol4 = (q(:,4) < q4lim(1) - tol) | (q(:,4) > q4lim(2) + tol);
        info.cabeceo.verificado     = true;
        info.cabeceo.ok             = ~any(viol4);
        info.cabeceo.filas_violadas = find(viol4);
        info.cabeceo.margen_deg     = min(min(q(:,4) - q4lim(1), ...
                                              q4lim(2) - q(:,4))) * 180/pi;
    end
end

% ---------- 10. Inercia de rotor reflejada (D14) -------------------------
M_rotor        = zeros(na);
rotor_incluido = false;
hay_nan_rotor  = false;
if isfield(P, 'act') && isfield(P.act, 'usar_actuadores') ...
        && islogical(P.act.usar_actuadores) && P.act.usar_actuadores
    if ~isfield(P.act, 'Jm_act') || ~isfield(P.act, 'G_act')
        error('PI7:mapeo:sinDatosRotor', ...
            ['P.act.usar_actuadores = true pero faltan P.act.Jm_act o ' ...
             'P.act.G_act (1 x %d). Ver D14.'], na);
    end
    Jm = P.act.Jm_act(:).';   G = P.act.G_act(:).';
    if numel(Jm) ~= na || numel(G) ~= na
        error('PI7:mapeo:tamDatosRotor', ...
            'P.act.Jm_act y P.act.G_act tienen que ser 1 x %d.', na);
    end
    M_rotor        = diag(Jm .* G.^2);       % [kg·m^2] NaN incluidos, a proposito
    rotor_incluido = true;
    hay_nan_rotor  = any(isnan(diag(M_rotor)));
else
    omitidas{end+1} = ['P.act.usar_actuadores = false: M_rotor = 0. La ' ...
        'ESTATICA es exacta igual; la dinamica con qdda distinto de cero ' ...
        'esta INCOMPLETA (falta Jm*G^2, que con reduccion puede igualar la ' ...
        'inercia del eslabon)'];
end
rotor_contamina = false;

% ---------- 11. Pares: articulares -> actuados ---------------------------
tau_a = [];
if nargin >= 3 && ~isempty(tau)
    if isvector(tau) && numel(tau) == 4
        tau = tau(:).';
    end
    if size(tau, 2) ~= 4
        error('PI7:mapeo:tamTau', ...
            ['tau tiene %d columnas y se espera N x 4: son los pares en ' ...
             'coordenadas ARTICULARES, tal cual salen de rne o gravload.'], ...
            size(tau, 2));
    end
    if size(tau, 1) == 1 && N > 1
        tau = repmat(tau, N, 1);
    elseif size(tau, 1) ~= N
        error('PI7:mapeo:filasTau', ...
            'tau tiene %d filas y qa tiene %d. Tienen que coincidir.', ...
            size(tau, 1), N);
    end

    tau_a = tau * dq_dqa;                    % N x na  ==  (dq_dqa' * tau')'

    if nargin >= 4 && ~isempty(qdda)
        if isvector(qdda) && numel(qdda) == na
            qdda = qdda(:).';
        end
        if size(qdda, 2) ~= na
            error('PI7:mapeo:tamQdda', ...
                'qdda tiene %d columnas y se esperan %d (actuadas).', ...
                size(qdda, 2), na);
        end
        if size(qdda, 1) == 1 && N > 1
            qdda = repmat(qdda, N, 1);
        elseif size(qdda, 1) ~= N
            error('PI7:mapeo:filasQdda', ...
                'qdda tiene %d filas y qa tiene %d.', size(qdda,1), N);
        end
        if ~isreal(qdda) || any(~isfinite(qdda(:)))
            error('PI7:mapeo:qddaNoFinito', ...
                'qdda contiene NaN, Inf o valores complejos.');
        end

        % --- Termino de rotor, elemento por elemento (D14, D16) ----------
        % Se evalua SOLO donde la aceleracion actuada es distinta de cero.
        % Con qdda_i = 0 exacto la contribucion del rotor i vale cero para
        % cualquier Jm y G finitos: un NaN que significa "todavia no
        % elegimos el motor" no tiene por que contaminar la estatica.
        % NO es sustituir NaN por cero: donde la aceleracion no es nula el
        % NaN se propaga tal cual y se reporta.
        Mr_diag = repmat(diag(M_rotor).', N, 1);      % N x na
        activo  = (qdda ~= 0);
        termino = zeros(N, na);
        termino(activo) = qdda(activo) .* Mr_diag(activo);
        tau_a = tau_a + termino;                      % [N·m]

        rotor_contamina = any(isnan(termino(:)));
        if rotor_contamina
            omitidas{end+1} = ['inercia de rotor NaN multiplicando una ' ...
                'aceleracion NO nula: tau_a sale NaN en esas juntas. Cargar ' ...
                'el datasheet del NEMA 17 y la relacion de transmision (P6)'];
        elseif hay_nan_rotor
            omitidas{end+1} = ['P.act.Jm_act / G_act tienen NaN, pero la ' ...
                'aceleracion actuada es nula donde hacen falta, asi que el ' ...
                'termino de rotor vale cero EXACTO y tau_a es exacto. No ' ...
                'confundir con dinamica verificada'];
        end

    elseif rotor_incluido
        omitidas{end+1} = ['tau_a devuelto SIN el termino de rotor: no se ' ...
            'paso qdda. Exacto para estatica, incompleto para dinamica'];
    end
end

% ---------- 12. Info -----------------------------------------------------
info.topologia_q4    = topo;
info.topologia_codo  = lower(codo);
info.topologia       = sprintf('%s / %s', topo, lower(codo));
info.na              = na;
info.nombres         = nombres;
info.dq_dqa          = dq_dqa;
info.dqa_dq          = dqa_dq;    % qa = q * dqa_dq' - offset_act (mapeo inverso)
info.offset_act_rad  = off_act_rad;
info.M_rotor         = M_rotor;
info.rotor_incluido  = rotor_incluido;
info.rotor_contamina = rotor_contamina;
info.tol             = tol;
info.omitidas        = omitidas;
info.ok              = info.topes.ok && info.cabeceo.ok;

end
