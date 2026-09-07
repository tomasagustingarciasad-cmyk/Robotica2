function P = PI7_gen_parametros_params()
% PI7_GEN_PARAMETROS_PARAMS  Única fuente de verdad numérica del Proyecto 3.
%
% QUÉ HACE : devuelve el struct P con toda la geometría, topes, masas, centros
%            de masa, inercias, actuadores, transmisión, esfera, plano
%            inclinado, datos de misión y opciones de simulación.
%            No calcula, no grafica, no imprime, no depende de toolboxes.
% RECIBE   : nada.
% DEVUELVE : P  (struct) — ver 04_CODIGO.md §3.1 para la organización.
% DEPENDE  : nada. MATLAB base. No requiere RTB.
% FUENTE   : 01_ESPECIFICACION.md §3.2–§3.6, §5, §6, §7, §8.
% AUTOR    : Equipo PI7 — Robótica II, UNCuyo.
% FECHA    : 2026-09-03  (rev. D13 accionamiento por lazos, D14 inercia de
%                         rotor en coordenadas actuadas, D15 topes invertidos)
%
% CONVENCIÓN DE VALOR AUSENTE:
%   NaN  -> magnitud numérica todavía sin definir por el equipo
%   ''   -> designación de componente sin definir
%   Los campos sin valor real están además listados en P.meta.pendientes.
%
% CONVENCIÓN DE UNIDADES: SI en todos los campos (m, kg, s, rad, N·m),
%   salvo los que llevan sufijo _deg, que están en grados.
%
% ADVERTENCIA: todo el bloque geométrico e inercial es PROVISORIO. Proviene
%   del avance rápido de las semanas 2–4 y no tiene respaldo en el CAD.

%% 0. Metadatos ----------------------------------------------------------
P.meta.proyecto      = 'PI7 - Robot serie para esfera y plano inclinado';
P.meta.nombre_robot  = 'PI7 Brazo 4 GDL';
P.meta.version       = 'R00';
P.meta.fecha         = '2026-09-03';
P.meta.provisorio    = true;                % geometría e inercias sin CAD

%% 1. Geometría (01_ESPECIFICACION §3.3) — TODO PROVISORIO ---------------
P.geom.L1 = 0.000;      % [m]  base -> eje del hombro.        PROVISORIO
P.geom.L2 = 0.150;      % [m]  eslabón brazo.                 PROVISORIO
P.geom.L3 = 0.150;      % [m]  eslabón antebrazo.             PROVISORIO
P.geom.L4 = 0.050;      % [m]  muñeca -> punto de agarre.     PROVISORIO
P.geom.zb = 0.050;      % [m]  altura del eje del hombro sobre la mesa. PROVISORIO
% NOTA: L1 = 0 y zb = 0,050 describen la misma distancia física por dos vías
% distintas (d1 de la DH y transformación de base). Hoy toda la altura la
% aporta T_base. Cuando el CAD la defina hay que elegir una sola vía.

% Tabla DH estándar (Corke). Columnas: [theta d a alpha].
% theta lo aporta la variable articular q; la columna 1 queda en cero.
P.geom.dh = [ 0,  P.geom.L1,  0,          pi/2 ;   % 1  base
              0,  0,          P.geom.L2,  0    ;   % 2  hombro
              0,  0,          P.geom.L3,  0    ;   % 3  codo
              0,  0,          P.geom.L4,  0    ];  % 4  cabeceo canasta

P.geom.offset_deg = [0 0 0 0];   % [deg] cero DH - cero mecánico ARTICULAR.
                                 %       Sin definir (P10)

P.geom.T_base = [eye(3), [0; 0; P.geom.zb]; 0 0 0 1];   % base -> hombro
P.geom.T_tool = eye(4);          % muñeca -> TCP. Sin definir (§3.2)

%% 1.1 Topología del mecanismo (D10, D11, D13) --------------------------
% Son DOS decisiones independientes. Cada una cambia una fila distinta de la
% matriz de mapeo. Ninguna se puede dejar implícita: la función de mapeo
% falla con error si alguna está vacía.
%
% (a) MUÑECA — cómo se gobierna el cabeceo de la canasta (D10):
%     'P1' paralelogramo pasivo puro, sin servo   -> q4 = -(q2+q3)
%     'P2' servo libre en la muñeca               -> q4 actuado directo
%     'P3' paralelogramo + servo en serie         -> q4 = -(q2+q3) + qs
P.geom.topologia_q4 = 'P3';        % DECIDIDA en D10

% (b) CODO — dónde está el motor que mueve el antebrazo (D13, cierra P11):
%     'junta'   motor sentado sobre el eje del codo -> comanda el ángulo
%               RELATIVO al brazo:  qa3 = q3
%     'L1_base' motor en la base, manivela en el eje del hombro, biela
%               paralela al brazo (lazo L1) -> comanda el ángulo ABSOLUTO
%               del antebrazo respecto de la base:  qa3 = q2 + q3
P.geom.topologia_codo = 'L1_base'; % DECIDIDA en D13: los tres NEMA 17 van
                                   % en la base. Ver 01_ESPECIFICACION §3.6

% CONDICIÓN DE CAD QUE SOSTIENE EL MAPEO: los tres NEMA 17 van montados
% sobre la COLUMNA QUE GIRA con q1, no sobre la base fija. Si fueran a la
% base fija, q2 y q3 quedarían además acoplados a q1 y la matriz de mapeo
% dejaría de ser la de §3.6.
P.geom.motores_sobre_columna = true;   % PROVISORIO, a confirmar con el CAD

% Sub-decisiones de D10, a confirmar contra el CAD (P12):
P.geom.par_ruteo      = 'barras';  % 'barras' | 'correas'. PROVISORIO.
                                   % Barras: ~0,08 deg de backlash por etapa.
                                   % Correas 1:1: 0,45–0,9 deg por etapa.
P.geom.par_tope_nivel = true;      % tope mecánico rígido que materializa la
                                   % canasta nivelada (D10, condición a). El
                                   % servo NO sostiene el nivel en transporte.

% Rango mecánico del cabeceo TOTAL q4. NO es un tope de actuador: es el
% límite de interferencia del pivote entre antebrazo y soporte de canasta.
% Se verifica sobre q4 CALCULADO, aparte de los topes de actuador.
% Signo: q4 > 0 retiene, q4 < 0 vuelca (00_DIRECTIVAS §3.2).
P.geom.q4_mec_deg = [-100, 20];    % [deg] PROVISORIO. Sale del CAD (P13).

%% 2. Actuadores y transmisión (01_ESPECIFICACION §8) -------------------

% --- INTERRUPTOR DE MODELO DE ACTUADORES (D09, revisado por D14) -------
% El SerialLink se arma SIEMPRE con Jm = 0 y G = 1: el modelo del brazo es
% el del mecanismo desnudo. La inercia de rotor reflejada NO puede vivir en
% el SerialLink porque el RTB la asocia una-a-una con cada articulación, y
% acá no hay un motor por articulación (D13). Entra en coordenadas ACTUADAS,
% dentro de PI7_gen_mapeo_actuadas_fun:
%
%     M_a = (dq/dqa)' * M * (dq/dqa) + diag(Jm_act .* G_act.^2)
%
% Este interruptor gobierna ese término, no el SerialLink:
%   false -> M_rotor = 0. Brazo desnudo. La ESTÁTICA es exacta igual.
%   true  -> usa P.act.Jm_act y P.act.G_act tal cual, NaN incluidos.
% No hay sustitución automática de NaN por cero.
P.act.usar_actuadores = false;

P.act.motor.modelo         = '';                % NEMA 17, modelo sin elegir (P6)
P.act.motor.paso_deg       = 1.8;               % [deg/paso] PROVISORIO, típico
P.act.motor.par_holding    = [NaN, NaN, NaN];   % [N·m] requiere datasheet
P.act.motor.corriente_fase = NaN;               % [A]   requiere datasheet
P.act.motor.Jm_rotor       = NaN;               % [kg·m^2] requiere datasheet
P.act.motor.tension        = NaN;               % [V]
P.act.motor.ubicacion      = 'columna base';    % D13: los tres en la base

P.act.driver.modelo     = '';   % A4988 / DRV8825 / TMC2209 sin elegir
P.act.driver.micropasos = NaN;  % [-]

P.act.trans.tipo         = 'correa dentada';   % D05, vigente
P.act.trans.G            = [NaN, NaN, NaN];    % [-] reducción por actuador (P6)
P.act.trans.rendimiento  = NaN;                % [-]
P.act.trans.backlash_deg = NaN;                % [deg]

% --- SERVO DEL CABECEO (D12: SG90 confirmado) --------------------------
% Elegido por MASA, no por par. El par estático del cabeceo es ~20,6 mN·m en
% el peor caso; el stall de catálogo del SG90 es ~177 mN·m.
P.act.servo.modelo    = 'SG90';        % D12, decidido
P.act.servo.masa      = 9.0e-3;        % [kg] catálogo
P.act.servo.par_stall = 0.177;         % [N·m] CATÁLOGO DE CLON, no datasheet.
                                       %       No usar como margen de diseño
                                       %       sin ensayo propio.
P.act.servo.Jm_rotor  = NaN;           % [kg·m^2] sin datasheet
P.act.servo.G         = NaN;           % [-]
% Recorrido útil de qs. El servo arranca APOYADO en el tope de nivel (D10,
% condición a) y solo se aleja de él para volcar. El tope está en qs = 0:
% el mecanismo bloquea todo qs > 0. Por eso el rango es [-90, 0] y no
% [-90, +10] como figuraba antes de D13.
P.act.servo.rango_deg    = [-90, 0];   % [deg] PROVISORIO, sale del CAD (P13)
P.act.servo.deadband_deg = NaN;        % [deg] sin medir
P.act.servo.backlash_deg = NaN;        % [deg] sin medir

P.act.fuente.tension   = NaN;   % [V] a dimensionar tras el cálculo dinámico
P.act.fuente.corriente = NaN;   % [A]

P.act.sensor.tipo        = '';    % final de carrera óptico o mecánico (P8)
P.act.sensor.usa_encoder = NaN;   % logical, sin decidir (P8)

%% 3. Coordenadas actuadas: nombres, topes y offsets (D13, D15) ---------
% CAMBIO DE DIRECCIÓN RESPECTO DE LA VERSIÓN ANTERIOR (D15):
% los topes FÍSICOS viven del lado ACTUADO — recorrido de cada manivela y del
% servo. Los topes ARTICULARES P.qlim_deg ya no son fuente de verdad: se
% DERIVAN de acá. Con el codo accionado por L1, q3 no tiene un tope propio,
% porque depende de dos motores a la vez.

% Rango del tercer actuador. Cambia de significado con la topología del codo.
switch upper(P.geom.topologia_codo)
    case 'JUNTA'
        qa3_rango_deg = [-90,  90];    % [deg] ángulo RELATIVO del codo
    case 'L1_BASE'
        qa3_rango_deg = [-90, 150];    % [deg] ángulo ABSOLUTO del antebrazo
                                       % PROVISORIO: recorrido de la manivela
    otherwise
        error('PI7:params:topologiaCodoInvalida', ...
            ['P.geom.topologia_codo = ''%s'' no es válida. ' ...
             'Admitidas: ''junta'', ''L1_base''.'], P.geom.topologia_codo);
end

qa1_rango_deg = [-180, 180];   % [deg] PROVISORIO. ±180 dudoso con correa y
                               %       cableado subiendo por la base
qa2_rango_deg = [   0, 150];   % [deg] PROVISORIO. Ángulo absoluto del brazo

switch upper(P.geom.topologia_q4)
    case 'P1'
        P.qlim_act_deg     = [qa1_rango_deg; qa2_rango_deg; qa3_rango_deg];
        P.meta.nombres_act  = {'q1', 'q2', 'q3abs'};
    case 'P2'
        P.qlim_act_deg     = [qa1_rango_deg; qa2_rango_deg; qa3_rango_deg; ...
                              P.geom.q4_mec_deg];
        P.meta.nombres_act  = {'q1', 'q2', 'q3abs', 'q4'};
    case 'P3'
        P.qlim_act_deg     = [qa1_rango_deg; qa2_rango_deg; qa3_rango_deg; ...
                              P.act.servo.rango_deg];
        P.meta.nombres_act  = {'q1', 'q2', 'q3abs', 'qs'};
    otherwise
        error('PI7:params:topologiaInvalida', ...
            ['P.geom.topologia_q4 = ''%s'' no es válida. ' ...
             'Admitidas: ''P1'', ''P2'', ''P3''.'], P.geom.topologia_q4);
end
if strcmpi(P.geom.topologia_codo, 'junta')
    P.meta.nombres_act{3} = 'q3';   % ángulo relativo: el nombre absoluto miente
end
na = size(P.qlim_act_deg, 1);

% Offset del cero de cada ACTUADOR respecto del cero del modelo (D13).
% El paralelogramo iguala el ángulo del balancín al de la manivela MÁS una
% constante de armado, y el cero de cada motor lo fija el sensor de homing.
% El mapeo es AFÍN, no lineal:   q = (qa + offset_act) * (dq/dqa)'
% Distinto de P.geom.offset_deg, que va del cero articular al cero DH.
P.geom.offset_act_deg = zeros(1, na);   % [deg] sin definir (P10)

%% 4. Topes articulares — DERIVADOS de los actuados (D15) ---------------
% ATENCIÓN: P.qlim_deg es una ENVOLVENTE EXTERIOR, no la caja realizable.
% La imagen de una caja en qa a través de un mapeo acoplado no es una caja
% en q: es un zonotopo. Acá se guarda su envolvente, que es más grande.
% Consecuencia: una postura puede estar dentro de P.qlim_deg y ser
% INCOMANDABLE. La verificación válida corre sobre qa y sobre q4, y vive en
% PI7_gen_mapeo_actuadas_fun. P.qlim_deg sirve para descartar ramas de la CI
% de forma conservadora (nunca descarta de más) y para el encuadre gráfico.
qa = P.qlim_act_deg;                        % [deg] atajo local
P.qlim_deg      = nan(4, 2);
P.qlim_deg(1,:) = qa(1,:);                  % q1 = qa1 siempre
P.qlim_deg(2,:) = qa(2,:);                  % q2 = qa2 siempre

switch upper(P.geom.topologia_codo)
    case 'JUNTA'
        P.qlim_deg(3,:) = qa(3,:);                          % q3 = qa3
    case 'L1_BASE'
        P.qlim_deg(3,:) = [qa(3,1) - qa(2,2), ...
                           qa(3,2) - qa(2,1)];              % q3 = qa3 - qa2
end

% q4 es un pivote FÍSICO (antebrazo <-> soporte de canasta): su tope es el
% de interferencia, no la envolvente del mapeo, que sería mucho más ancha.
P.qlim_deg(4,:) = P.geom.q4_mec_deg;
clear qa qa1_rango_deg qa2_rango_deg qa3_rango_deg

%% 5. Inercia de rotor en coordenadas actuadas (D14) --------------------
% Un elemento por ACTUADOR, no por articulación. Convención RTB:
% w_motor = G * w_actuador, inercia reflejada = Jm * G^2.
if na == 3
    P.act.Jm_act = P.act.motor.Jm_rotor * [1 1 1];              % [kg·m^2]
    P.act.G_act  = P.act.trans.G;                               % [-]
else
    P.act.Jm_act = [P.act.motor.Jm_rotor * [1 1 1], P.act.servo.Jm_rotor];
    P.act.G_act  = [P.act.trans.G,                  P.act.servo.G       ];
end

%% 6. Masas (01_ESPECIFICACION §7) — PROVISORIAS, SIN MOTORES -----------
P.masa.eslabon = [74.4, 74.4, 24.8, 18.4] * 1e-3;   % [kg] PROVISORIO
P.masa.link    = P.masa.eslabon;    % lo que se carga en el SerialLink

P.masa.motor       =  [0.280, 0.280, 0.280];   % [kg] NEMA 17. 0,22–0,35 kg típico (P6)
P.masa.servo       = P.act.servo.masa;  % [kg] SG90, en la muñeca (D12)
P.masa.transmision = [NaN, NaN, NaN];   % [kg] correas, poleas, ejes, rodamientos
P.masa.incluye_motores = true;

% D13: los tres NEMA 17 van sobre la COLUMNA que gira con q1, o sea sobre el
% eslabón 1. No hacen palanca gravitatoria (están sobre el eje vertical) y
% por eso no afectan la estática, pero SÍ dominan:
%   - la inercia respecto de z0, y con ella el par dinámico de q1;
%   - la carga sobre el rodamiento de base.
% Mientras P.masa.link(1) valga 74,4 g el par dinámico de q1 es optimista
% por un factor grande. El constructor lo avisa.
P.masa.motor_en_eslabon = [1, 1, 1];    % D13: los tres sobre el eslabón 1

% Bielas de los tres lazos (D11) y pieza de triple pivote del codo. NO son
% cuerpos aparte del modelo: se reparten sobre los eslabones 2 y 3 cuando el
% CAD las defina (P12). Estimación gruesa: ~9e-3 kg cada biela.
P.masa.biela    = [9e-3, 9e-3, 9e-3];      % [kg] bielas de L1, L2, L3
P.masa.triplate = 15e-3;                  % [kg] pieza de triple pivote del codo

% Agrupación en los eslabones cinemáticos:
P.masa.link = P.masa.eslabon;
P.masa.link(1) = P.masa.link(1) + sum(P.masa.motor);
P.masa.link(2) = P.masa.link(2) + P.masa.biela(1) + P.masa.biela(2) + P.masa.triplate;
P.masa.link(3) = P.masa.link(3) + P.masa.biela(3);

%% 7. Centros de masa (01_ESPECIFICACION §7) — PROVISORIOS --------------
% Barras esbeltas homogéneas, expresadas en el marco DH del propio eslabón.
% SIGNO: en DH estándar el origen del marco i está en el extremo DISTAL del
% eslabón i, y el cuerpo se extiende hacia -x_i. El CoM va en -a_i/2.
% Poner +a_i/2 (código legado) triplica el brazo de palanca gravitatorio.
%
% RESTRICCIÓN DE CAD (D10, condición b): el eje de cabeceo q4 va POR DELANTE
% del CoM de la canasta, vacía y cargada, para que la gravedad precargue
% permanentemente contra el tope de nivel.
P.com.r = [  0,             0, 0 ;
            -P.geom.L2/2,   0, 0 ;
            -P.geom.L3/2,   0, 0 ;
            -P.geom.L4/2,   0, 0 ];   % [m]  PROVISORIO

%% 8. Tensores de inercia (01_ESPECIFICACION §7) — PROVISORIOS ---------
% Barra esbelta respecto de su CoM, longitud sobre x del marco DH.
Lg = [P.geom.L1, P.geom.L2, P.geom.L3, P.geom.L4];   % [m]
P.inercia.I = zeros(3,3,4);                          % [kg·m^2]
% ESLABÓN 1: valor de relleno. Con los tres NEMA 17 montados sobre la
% columna (D13) la inercia real respecto de z0 es de orden 1e-3 kg·m^2, tres
% órdenes por encima de esto. Todo par dinámico de q1 calculado con este
% valor es ficticio. Sale del CAD.
P.inercia.I(:,:,1) = diag([1e-3, 1e-3, 1e-3]);       % PROVISORIO / RELLENO
for i = 2:4
    Ib = P.masa.eslabon(i) * Lg(i)^2 / 12;           % [kg·m^2]
    P.inercia.I(:,:,i) = diag([1e-6, Ib, Ib]);       % PROVISORIO
end
clear i Ib Lg na

%% 9. Fricción (01_ESPECIFICACION §7.1, decisión D07) -------------------
P.fric.usar        = false;      % D07: el modelo principal ignora la fricción
P.fric.B           = [0 0 0 0];  % [N·m·s/rad] viscosa
P.fric.Tc          = zeros(4,2); % [N·m] Coulomb [positiva, negativa]
P.fric.Tc_estimado = 1.4e-3;     % [N·m] estimación del legado. Solo referencia

%% 10. Esfera (01_ESPECIFICACION §5) — MATERIAL SIN DEFINIR (P4) --------
P.esfera.diametro      = 0.020;   % [m] requisito de la cátedra
P.esfera.radio         = P.esfera.diametro/2;                  % [m]
P.esfera.volumen       = (4/3)*pi*P.esfera.radio^3;            % [m^3]
P.esfera.material      = 'acero';      % sin definir
P.esfera.densidad      = 7800;     % [kg/m^3]
P.esfera.masa          = P.esfera.volumen*P.esfera.densidad;     % [kg]
P.esfera.I             = (2/5) * 32.7e-3 * P.esfera.radio^2;     % [kg·m^2] esfera maciza: (2/5)·m·r^2
P.esfera.e_restitucion = 0.6;     % [-] acero/PLA ~0,6 según §6.3
P.esfera.mu_s          = NaN;     % [-] fricción estática contra el plano
% Posición del centro de la esfera en el marco 4, necesaria para cargar el
% payload del SerialLink. Sale de la geometría de la canasta (P2).
P.esfera.r_efector     = [0, 0, 0];  % [m] Provisorio, origen en marco 4

% Candidatos evaluados en §5. Datos, no decisión.
P.esfera.candidatos = struct( ...
    'material', {'acero','vidrio','PLA macizo','nylon/POM'}, ...
    'densidad', {7800, 2500, 1240, 1150}, ...          % [kg/m^3]
    'masa',     {32.7e-3, 10.5e-3, 5.2e-3, 4.8e-3});   % [kg] para D = 20 mm

%% 11. Efector: retención (D10) — SIN DEFINIR --------------------------
% Ángulo de retención de la canasta: inclinación máxima de la gravedad
% efectiva que la esfera tolera sin salirse.
%     a_max = g * tan(phi_ret - phi_err)
P.efector.phi_ret_deg = NaN;   % [deg] sale de la geometría de la canasta (P2)
P.efector.phi_err_deg = NaN;   % [deg] presupuesto de error de nivelación

%% 12. Plano inclinado (01_ESPECIFICACION §6) — SIN DEFINIR (P5, P9) ---
P.plano.beta_deg          = NaN;        % [deg] inclinación de operación
P.plano.longitud          = NaN;        % [m]
P.plano.altura_caida      = NaN;        % [m] h de §6.2
P.plano.mu_s              = NaN;        % [-] exige mu_s >= (2/7)·tan(beta)
P.plano.material          = '';
P.plano.canaleta_ancho    = NaN;        % [m]
P.plano.regulable         = true;       % exigencia de la pre-BOM
P.plano.T_robot_plano     = NaN(4,4);   % pose del plano respecto de la base (P9)
P.plano.modo_recuperacion = '';         % 'tope'|'espera_quieta'|'en_movimiento'

%% 13. Misión (01_ESPECIFICACION §1) — SIN DEFINIR ---------------------
% Las posturas van en coordenadas ACTUADAS qa: son lo que se le comanda al
% robot. Con P3 + L1_base, qa = [q1, q2, q3abs, qs] (D13).
P.mision.t_ciclo        = NaN;        % [s]
P.mision.qa_home_deg    = NaN(1,4);   % [deg]
P.mision.qa_vuelco_deg  = NaN(1,4);   % [deg] liberación sobre la canaleta
P.mision.qa_espera_deg  = NaN(1,4);   % [deg]
P.mision.qa_captura_deg = NaN(1,4);   % [deg] recuperación
P.mision.qd_max_deg     = NaN(1,4);   % [deg/s]   sale del par-velocidad
P.mision.qdd_max_deg    = NaN(1,4);   % [deg/s^2]
P.mision.jerk_max_deg   = NaN(1,4);   % [deg/s^3] estrategia de control (D06)
P.mision.v_max_tcp      = NaN;        % [m/s]
P.mision.a_max_tcp      = NaN;        % [m/s^2]

%% 14. Simulación -------------------------------------------------------
P.sim.g        = 9.81;              % [m/s^2]
P.sim.gravedad = [0; 0; 9.81];      % [m/s^2] convención RTB (D08): es la
                                    % aceleración impuesta a la base, no el
                                    % vector campo. Signo POSITIVO.
P.sim.tol_pos       = 1e-4;         % [m]   criterio CD<->CI, 00_DIRECTIVAS §7.2
P.sim.tol_ang       = 1e-4;         % [rad] PROVISORIO
P.sim.tol_topes_rad = 1e-9;         % [rad] margen de punto flotante, no físico
P.sim.n_muestras    = 50;          % [-]   PROVISORIO
P.sim.dt            = 1e-3;         % [s]   PROVISORIO
P.sim.workspace     = [-0.40 0.40 -0.40 0.40 -0.05 0.50];  % [m] encuadre

%% 15. Inventario de campos sin valor real ------------------------------
% usar_actuadores NO está acá: tiene valor real, es un interruptor de modelo.
% geom.topologia_q4 y geom.topologia_codo salieron de la lista: D10 y D13.
P.meta.pendientes = { ...
    'geom.offset_deg',    'geom.T_tool',       'geom.offset_act_deg', ...
    'masa.motor',         'masa.transmision', ...
    'masa.biela',         'masa.triplate', ...
    'act.motor.modelo',   'act.motor.par_holding', 'act.motor.corriente_fase', ...
    'act.motor.Jm_rotor', 'act.motor.tension', ...
    'act.driver.modelo',  'act.driver.micropasos', ...
    'act.trans.G',        'act.trans.rendimiento', 'act.trans.backlash_deg', ...
    'act.servo.Jm_rotor', 'act.servo.G', ...
    'act.servo.deadband_deg', 'act.servo.backlash_deg', ...
    'act.fuente.tension', 'act.fuente.corriente', ...
    'act.sensor.tipo',    'act.sensor.usa_encoder', ...
    'act.Jm_act',         'act.G_act', ...
    'esfera.material',    'esfera.densidad',   'esfera.masa', ...
    'esfera.I',           'esfera.e_restitucion', 'esfera.mu_s', ...
    'esfera.r_efector', ...
    'efector.phi_ret_deg','efector.phi_err_deg', ...
    'plano.beta_deg',     'plano.longitud',    'plano.altura_caida', ...
    'plano.mu_s',         'plano.material',    'plano.canaleta_ancho', ...
    'plano.T_robot_plano','plano.modo_recuperacion', ...
    'mision.t_ciclo',     'mision.qa_home_deg', 'mision.qa_vuelco_deg', ...
    'mision.qa_espera_deg','mision.qa_captura_deg', ...
    'mision.qd_max_deg',  'mision.qdd_max_deg','mision.jerk_max_deg', ...
    'mision.v_max_tcp',   'mision.a_max_tcp' };

% Campos CON valor pero PROVISORIO (existen, no bloquean, no son confiables).
P.meta.provisorios = { ...
    'geom.L1','geom.L2','geom.L3','geom.L4','geom.zb','geom.dh', ...
    'geom.q4_mec_deg','geom.par_ruteo','geom.motores_sobre_columna', ...
    'qlim_act_deg','qlim_deg','masa.eslabon','masa.link','com.r','inercia.I', ...
    'act.motor.paso_deg','act.servo.par_stall','act.servo.rango_deg', ...
    'sim.tol_ang','sim.n_muestras','sim.dt' };

end
