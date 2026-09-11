function [R, avisos] = PI7_gen_construir_robot_fun(P)
% PI7_GEN_CONSTRUIR_ROBOT_FUN  Arma el SerialLink del PI7 a partir de P.
%
% QUÉ HACE : construye el objeto SerialLink de 4 eslabones con la DH estándar,
%            topes, offsets, base, tool, masas, centros de masa, tensores de
%            inercia y fricción, tomando TODO de P. Carga el payload de la
%            esfera si está definido.
%            No calcula, no grafica, no imprime.
% RECIBE   : P (struct) — salida de PI7_gen_parametros_params.
% DEVUELVE : R      (SerialLink) modelo del MECANISMO DESNUDO.
%            avisos (cell de char) qué quedó fuera del modelo y qué
%                   limitaciones tiene el resultado dinámico.
% DEPENDE  : Robotics Toolbox (Peter Corke) 10.4 — Link, SerialLink.
% AUTOR    : Equipo PI7 — Robótica II, UNCuyo.
% FECHA    : 2026-09-03  (rev. D13, D14)
%
% TOPOLOGÍA: el modelo se arma SIEMPRE con los cuatro eslabones
%   independientes y la misma tabla DH, válido para todas las combinaciones
%   de topología de muñeca (P1/P2/P3, D10) y de accionamiento del codo
%   ('junta'/'L1_base', D13). Este archivo NO conoce la topología.
%   Las restricciones q3 = qa3 - qa2 y q4 = -(q2+q3) + qs NO se imponen acá:
%   viven en PI7_gen_mapeo_actuadas_fun, junto con su dual sobre los pares.
%   Todo torque devuelto por R.rne está en coordenadas ARTICULARES y no es
%   directamente el par de ningún actuador.
%
% INERCIA DE ROTOR (D14 — CAMBIO RESPECTO DE D09):
%   Este constructor carga SIEMPRE L.Jm = 0 y L.G = 1. El RTB asocia esos
%   campos una-a-una con cada articulación, y en el PI7 no hay un motor por
%   articulación: los tres NEMA 17 están en la base y accionan por lazos
%   (D13). La reducción del motor del codo se refiere a su propia manivela,
%   no a q3. Cargarla como si fuera de q3 da un resultado plausible y
%   equivocado, visible sólo cuando el brazo acelera.
%   El término Jm·G^2 entra en coordenadas ACTUADAS, dentro de
%   PI7_gen_mapeo_actuadas_fun. El interruptor P.act.usar_actuadores gobierna
%   ese término, no este archivo.
%   Efecto colateral bueno: con Jm = 0 el modelo nunca devuelve NaN por falta
%   de datasheet, así que la estática corre siempre.

    %% Validación de lo que sí es indispensable -------------------------
    campos = {'geom','qlim_deg','qlim_act_deg','masa','com','inercia', ...
              'act','fric','sim','meta','esfera','mision'};
    for k = 1:numel(campos)
        if ~isfield(P, campos{k})
            error('PI7:construirRobot:faltaCampo', ...
                  'P no tiene el campo "%s". Revisar PI7_gen_parametros_params.', campos{k});
        end
    end
    if ~isfield(P.act, 'usar_actuadores') || ~islogical(P.act.usar_actuadores)
        error('PI7:construirRobot:interruptor', ...
              'P.act.usar_actuadores tiene que existir y ser logical.');
    end

    n = size(P.geom.dh, 1);
    if n ~= 4
        error('PI7:construirRobot:dhTamano', 'P.geom.dh debe tener 4 filas, tiene %d.', n);
    end
    if any(isnan(P.geom.dh(:)))
        error('PI7:construirRobot:dhNaN', ...
              'P.geom.dh contiene NaN: la cinemática no se puede construir.');
    end
    if any(P.qlim_deg(:,1) >= P.qlim_deg(:,2))
        error('PI7:construirRobot:qlim', 'Algún tope articular tiene mínimo >= máximo.');
    end

    %% Construcción de los eslabones ------------------------------------
    L = Link.empty(0, n);
    for i = 1:n
        L(i) = Link( 'd',      P.geom.dh(i,2), ...
                     'a',      P.geom.dh(i,3), ...
                     'alpha',  P.geom.dh(i,4), ...
                     'offset', deg2rad(P.geom.offset_deg(i)), ...
                     'revolute', 'standard');

        L(i).qlim = deg2rad(P.qlim_deg(i,:));   % [rad] envolvente exterior (D15)

        % --- parámetros inerciales del eslabón ---
        L(i).m = P.masa.link(i);            % [kg]      PROVISORIO, sin motores
        L(i).r = P.com.r(i,:);              % [m]       CoM en el marco DH
        L(i).I = P.inercia.I(:,:,i);        % [kg·m^2]  respecto del CoM

        % --- actuador: SIEMPRE neutro (D14) ---
        L(i).Jm = 0;                        % [kg·m^2]  ver cabecera
        L(i).G  = 1;                        % [-]

        % --- fricción (D07: desactivada) ---
        L(i).B  = P.fric.B(i);              % [N·m·s/rad]
        L(i).Tc = P.fric.Tc(i,:);           % [N·m] [positiva, negativa]
    end

    %% Ensamble del robot ------------------------------------------------
    R = SerialLink(L, 'name', P.meta.nombre_robot);
    R.base    = P.geom.T_base;
    R.tool    = P.geom.T_tool;
    R.gravity = P.sim.gravedad;             % [m/s^2] convención RTB, D08
    R.plotopt = {'workspace', P.sim.workspace, 'nobase', 'noshadow'};

    %% Payload de la esfera (problema 13 de 02_ESTADO) -------------------
    avisos = {};
    esfera_cargada = false;
    if ~isnan(P.esfera.masa) && ~any(isnan(P.esfera.r_efector))
        R.payload(P.esfera.masa, P.esfera.r_efector(:));
        esfera_cargada = true;
    end

    %% Etiquetas ----------------------------------------------------------
    if isempty(P.geom.topologia_q4), t4 = 'sin definir'; else, t4 = P.geom.topologia_q4; end
    if ~isfield(P.geom,'topologia_codo') || isempty(P.geom.topologia_codo)
        tc = 'sin definir';
    else
        tc = P.geom.topologia_codo;
    end
    R.comment = sprintf(['Topologia %s / codo %s | Jm=0 G=1 (rotor en coord. ' ...
        'actuadas, D14) | torques en coord. articulares'], t4, tc);

    %% Avisos -------------------------------------------------------------
    avisos{end+1} = ['Modelo del MECANISMO DESNUDO: L.Jm = 0 y L.G = 1 en las ' ...
        'cuatro juntas (D14). La inercia de rotor reflejada NO está en R: entra ' ...
        'en coordenadas actuadas, en PI7_gen_mapeo_actuadas_fun. gravload y la ' ...
        'estática son EXACTOS igual; la dinámica con qdd distinto de cero exige ' ...
        'pasar por el mapeo con qdda para completarse.'];

    if ~P.act.usar_actuadores
        avisos{end+1} = ['P.act.usar_actuadores = false: el mapeo devolverá ' ...
            'M_rotor = 0. No presentar torques dinámicos así obtenidos como el ' ...
            'modelo dinámico R00 que pide la cátedra.'];
    elseif isfield(P.act,'Jm_act') && (any(isnan(P.act.Jm_act)) || any(isnan(P.act.G_act)))
        avisos{end+1} = ['P.act.usar_actuadores = true con NaN en Jm_act o ' ...
            'G_act: la inercia de rotor reflejada saldrá NaN en el mapeo. Cargar ' ...
            'el datasheet del NEMA 17 y la relación de transmisión (P6).'];
    end

    if ~P.masa.incluye_motores
        avisos{end+1} = ['P.masa.link no incluye motores, correas, poleas ni las ' ...
            'bielas de los tres lazos: los torques corresponden a un brazo ' ...
            'desnudo y subestiman la carga real.'];
    end

    % D13: los tres NEMA 17 van sobre la columna que gira con q1.
    if isfield(P.masa,'motor') && any(isnan(P.masa.motor))
        avisos{end+1} = ['Los tres NEMA 17 van sobre el eslabón 1 (columna que ' ...
            'gira con q1, D13) y no están cargados. No afecta la ESTÁTICA — están ' ...
            'sobre el eje vertical y no hacen palanca gravitatoria — pero el par ' ...
            'DINÁMICO de q1 y la carga del rodamiento de base salen optimistas ' ...
            'por un factor grande: P.masa.link(1) = %.1f g contra 0,7–1,0 kg de ' ...
            'motores.'];
        avisos{end} = sprintf(avisos{end}, P.masa.link(1)*1e3);
    end

    if trace(P.inercia.I(:,:,1)) < 1e-5
        avisos{end+1} = ['P.inercia.I(:,:,1) es un valor de relleno (1e-6). Con ' ...
            'los tres motores sobre la columna la inercia real respecto de z0 es ' ...
            'de orden 1e-3 kg·m^2. Todo par dinámico de q1 calculado con esto es ' ...
            'ficticio.'];
    end

    if ~esfera_cargada
        if isnan(P.esfera.masa)
            falta = 'P.esfera.masa (material sin elegir, P4)';
        else
            falta = 'P.esfera.r_efector (posición del centro en el marco 4, P2)';
        end
        avisos{end+1} = sprintf(['Payload NO cargado: falta %s. El ítem del ' ...
            'checklist "modelo dinámico con masa de la esfera y del efector ' ...
            'final" queda insatisfecho.'], falta);
    else
        avisos{end+1} = sprintf(['Payload cargado: %.1f g en [%.1f %.1f %.1f] mm ' ...
            'del marco 4.'], P.esfera.masa*1e3, P.esfera.r_efector*1e3);
    end

    if any(isnan(P.mision.qd_max_deg)) || isnan(P.mision.t_ciclo)
        avisos{end+1} = ['P.mision sin límites de velocidad ni tiempo de ciclo: ' ...
            'cualquier trayectoria que alimente rne es arbitraria (problema 9).'];
    end

    if isfield(P.geom,'motores_sobre_columna') && ~P.geom.motores_sobre_columna
        avisos{end+1} = ['P.geom.motores_sobre_columna = false: si los motores no ' ...
            'giran con q1, q2 y q3 quedan además acoplados a q1 y la matriz de ' ...
            'mapeo de D13 deja de valer.'];
    end

    if P.meta.provisorio
        avisos{end+1} = 'Geometría, topes, masas, CoM e inercias son PROVISORIOS, sin respaldo de CAD.';
    end
end
