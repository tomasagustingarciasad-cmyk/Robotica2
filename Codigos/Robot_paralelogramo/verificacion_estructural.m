%% ========================================================================
%  VERIFICACION ESTRUCTURAL - Robot serie para esfera y plano inclinado
%  Robotica II - Proyecto 3 - UNCUYO
%
%  Cubre los pasos 03 a 09 del borrador de calculo:
%    03  Seccion (maciza, hipotesis de catedra)
%    04  Verificacion a tension
%    05  Verificacion a flecha
%    06  Pandeo de la biela
%    07  Par del servo de pitch
%    08  Pares resultantes en los motores
%    09  Impacto y requisito del sistema de amortiguacion
%
%  Hipotesis: seccion maciza 100% de relleno, carga estatica.
%  El impacto NO se usa para dimensionar: se cuantifica para generar el
%  requisito del amortiguador (indicacion de catedra).
% =========================================================================

clear; clc; close all;
% diary('resultados.txt');   % descomentar para volcar la salida a un archivo

%% ------------------------- DATOS DE ENTRADA -----------------------------
g        = 9.81;        % m/s^2
L        = 150;         % mm   longitud de cada eslabon
b        = 20;          % mm   ancho de la seccion
h        = 20;          % mm   altura de la seccion
r_pal    = 40;          % mm   radio de la palanca del paralelogramo
N_red    = 4.5;         % -    reduccion por correa (90/20 dientes)

% Material: PLA impreso
E_pla    = 3000;        % MPa  modulo de elasticidad
sig_rot  = 45;          % MPa  tension de rotura
FS       = 3;           % -    coeficiente de seguridad adoptado
rho_pla  = 1.24e-3;     % g/mm^3

% Masas en la punta [g]
m_bolilla = 32.9;       % bolilla de rulemen de acero D20
m_canasta = 15.0;
m_servo   = 13.5;       % MG90S
m_herraje = 10.0;
m_biela   = 45.0;       % g   masa de la biela (pleuel)

% Actuadores
T_nema    = 0.40;       % N.m  par de retencion del NEMA 17
T_servo   = 0.216;      % N.m  MG90S a 6 V (2,2 kgf.cm)
d_cg_can  = 25;         % mm   CG de canasta+bolilla al eje del servo

% Condiciones de captura
v_bolilla = 0.5;        % m/s  velocidad de la bolilla al llegar
t_frenado = 0.010;      % s    tiempo de frenado supuesto (solo para el servo;
                        %      la viga se resuelve por energia, en el paso 09)

sig_adm  = sig_rot / FS;                    % MPa
m_punta  = m_bolilla + m_canasta + m_servo + m_herraje;   % g
P        = m_punta/1000 * g;                % N

fprintf('=========================================================\n');
fprintf(' VERIFICACION ESTRUCTURAL - eslabones %gx%g x %g mm, PLA\n', b, h, L);
fprintf(' Hipotesis: seccion maciza (100%% de relleno), carga estatica\n');
fprintf('=========================================================\n\n');
fprintf('Carga en la punta: %.1f g  ->  P = %.3f N\n', m_punta, P);
fprintf('Tension admisible: %.0f/%.0f = %.1f MPa\n\n', sig_rot, FS, sig_adm);

%% ---------------------- 03  SECCION ------------------------------------
A     = b*h;                    % mm^2
I     = b*h^3/12;               % mm^4
c     = h/2;                    % mm
W     = I/c;                    % mm^3
m_esl = A*L*rho_pla;            % g   masa de un eslabon
q     = (m_esl/1000)*g / L;     % N/mm  peso propio repartido

fprintf('--- 03  SECCION ----------------------------------------\n');
fprintf('  A = b*h            = %8.0f mm^2\n', A);
fprintf('  I = b*h^3/12       = %8.0f mm^4\n', I);
fprintf('  W = I/c            = %8.0f mm^3\n', W);
fprintf('  masa del eslabon   = %8.1f g\n', m_esl);
fprintf('  q = m*g/L          = %8.4f N/mm\n\n', q);

%% ------------- MOMENTO FLECTOR (base de 04 y 05) -----------------------
M_P = P*L;                      % N.mm  contribucion de la carga puntual
M_q = q*L^2/2;                  % N.mm  contribucion del peso propio
M   = M_P + M_q;                % N.mm  momento maximo, en el codo

fprintf('--- MOMENTO FLECTOR MAXIMO (empotramiento) -------------\n');
fprintf('  M = P*L + q*L^2/2 = %.1f + %.1f = %.1f N.mm\n', M_P, M_q, M);
fprintf('  (el peso propio aporta el %.0f%% del total)\n\n', 100*M_q/M);

%% ---------------------- 04  TENSION ------------------------------------
sigma = M/W;
n_sig = sig_adm/sigma;

fprintf('--- 04  TENSION ----------------------------------------\n');
fprintf('  sigma = M/W = %.1f/%.0f = %.3f MPa\n', M, W, sigma);
fprintf('  n = %.1f/%.3f = %.0f   -> %s\n\n', sig_adm, sigma, n_sig, veredicto(n_sig>=FS));

%% ---------------------- 05  FLECHA -------------------------------------
EI    = E_pla*I;                % N.mm^2
d_P   = P*L^3/(3*EI);           % mm
d_q   = q*L^4/(8*EI);           % mm
delta = d_P + d_q;              % mm  flecha de un eslabon
delta_tot = 2*delta;            % mm  los dos eslabones en serie
res_robot = deg2rad(1.8/N_red)*2*L;   % mm  resolucion del robot en la punta

fprintf('--- 05  FLECHA -----------------------------------------\n');
fprintf('  EI = %.3e N.mm^2\n', EI);
fprintf('  delta = P*L^3/(3EI) + q*L^4/(8EI) = %.4f + %.4f = %.3f mm\n', d_P, d_q, delta);
fprintf('  Los dos eslabones en serie: %.3f mm\n', delta_tot);
fprintf('  Resolucion del robot en la punta: %.2f mm  (%.0fx mayor)\n', res_robot, res_robot/delta_tot);
fprintf('  Diametro de la bolilla: 20 mm  (la flecha es el %.1f%%)  -> %s\n\n', ...
        100*delta_tot/20, veredicto(delta_tot < 0.1*20));

%% ---------------------- 06  PANDEO DE LA BIELA -------------------------
F_biela_dis = 10;               % N   fuerza de diseno adoptada con margen
secciones   = [10 3; 12 4; 15 4; 15 5];   % [ancho espesor] mm

fprintf('--- 06  PANDEO DE LA BIELA (Euler, bi-articulada) ------\n');
fprintf('  P_cr = pi^2*E*I/L^2      con F de diseno = %.0f N\n', F_biela_dis);
fprintf('  %-12s %10s %10s %8s\n', 'seccion', 'I [mm^4]', 'Pcr [N]', 'n');
for k = 1:size(secciones,1)
    bb = secciones(k,1);  tt = secciones(k,2);
    I_b  = bb*tt^3/12;                      % eje debil: el espesor al cubo
    Pcr  = pi^2*E_pla*I_b/L^2;
    fprintf('  %2dx%d mm      %10.1f %10.1f %8.1f\n', bb, tt, I_b, Pcr, Pcr/F_biela_dis);
end
fprintf('  ADOPTADA: 12x4 mm\n\n');

%% ---------------------- 07  SERVO DE PITCH -----------------------------
m_carga  = (m_bolilla + m_canasta)/1000;            % kg
T_s_est  = m_carga*g*(d_cg_can/1000);               % N.m  estatico
p_bolilla= (m_bolilla/1000)*v_bolilla;              % N.s  cantidad de movimiento
F_imp_s  = p_bolilla/t_frenado;                     % N
T_s_imp  = F_imp_s*(d_cg_can/1000);                 % N.m

fprintf('--- 07  SERVO DE PITCH ---------------------------------\n');
fprintf('  Estatico: T = %.1f g * g * %.0f mm = %.4f N.m = %.2f kgf.cm\n', ...
        m_carga*1000, d_cg_can, T_s_est, T_s_est*10.197);
fprintf('  Impacto : p = m*v = %.4f N.s, frena en %.0f ms -> F = %.2f N\n', ...
        p_bolilla, t_frenado*1000, F_imp_s);
fprintf('            T = %.4f N.m\n', T_s_imp);
fprintf('  MG90S = %.3f N.m  ->  n = %.0f (impacto)  -> %s\n\n', ...
        T_servo, T_servo/T_s_imp, veredicto(T_servo/T_s_imp >= 2));

%% ---------------------- 08  PARES EN LOS MOTORES -----------------------
[T2, T3, F_biela] = pares(P, q, L, r_pal, m_esl, m_biela, g);

m_movil = 2*m_esl + m_punta + m_biela;

fprintf('--- 08  PARES EN LOS MOTORES (estatico) ----------------\n');
fprintf('  Momento en el codo    M_A = %.1f N.mm\n', T3*1000);
fprintf('  Fuerza en la biela    F   = %.2f N (traccion)\n', F_biela);
fprintf('  Palanca / codo        T3  = %.3f N.m  ->  motor %.4f N.m  (margen %.1fx)\n', ...
        T3, T3/N_red, T_nema/(T3/N_red));
fprintf('  Hombro                T2  = %.3f N.m  ->  motor %.4f N.m  (margen %.1fx)\n', ...
        T2, T2/N_red, T_nema/(T2/N_red));
fprintf('  Masa movil total          = %.0f g\n', m_movil);
fprintf('  -> %s\n\n', veredicto(min(T_nema./([T2 T3]/N_red)) >= 2));

%% ---------------------- 09  IMPACTO Y AMORTIGUACION --------------------
Ec    = 0.5*(m_bolilla/1000)*v_bolilla^2;   % J
Ec_mm = Ec*1000;                            % N.mm
k_viga= 3*EI/L^3;                           % N/mm  rigidez de la punta
d_imp = sqrt(2*Ec_mm/k_viga);               % mm
F_imp = k_viga*d_imp;                       % N
FD    = F_imp/P;                            % factor dinamico

M_imp   = F_imp*L + M_q;
sig_imp = M_imp/W;
[T2i, T3i] = pares(F_imp, q, L, r_pal, m_esl, m_biela, g);

fprintf('--- 09  IMPACTO (no se disena contra el: se cuantifica) -\n');
fprintf('  E = 1/2*m*v^2 = %.4f J = %.2f N.mm\n', Ec, Ec_mm);
fprintf('  k = 3EI/L^3 = %.1f N/mm\n', k_viga);
fprintf('  1/2*k*d^2 = E  ->  d = %.3f mm  ->  F = %.1f N\n', d_imp, F_imp);
fprintf('  FACTOR DINAMICO = %.0fx\n', FD);
fprintf('  Eslabon : sigma = %.2f MPa -> n = %.1f  -> %s\n', ...
        sig_imp, sig_adm/sig_imp, veredicto(sig_adm/sig_imp >= FS));
fprintf('  Motores : T3 = %.2f N.m -> motor %.3f N.m\n', T3i, T3i/N_red);
fprintf('            T2 = %.2f N.m -> motor %.3f N.m\n', T2i, T2i/N_red);
fprintf('            NEMA 17 retiene %.2f N.m  -> %s\n', T_nema, ...
        veredicto(max([T2i T3i]/N_red) <= T_nema));
% Fuerza en la punta a la que cada motor llega a su par de retencion.
% T2 y T3 son lineales en P, asi que se despeja directo:
F_lim_T2 = (T_nema*N_red*1000 - ((m_esl/1000)*g*L/2 + (m_esl/1000)*g*L ...
            + (m_biela/1000)*g*L/2)) / L;
F_lim_T3 = (T_nema*N_red*1000 - q*L^2/2) / L;
F_lim    = min(F_lim_T2, F_lim_T3);
fprintf('  El primer motor pierde pasos a partir de F = %.1f N en la punta\n', F_lim);
fprintf('  Sin encoders, perder pasos = perder la posicion sin detectarlo.\n\n');

fprintf('--- REQUISITO DEL SISTEMA DE AMORTIGUACION -------------\n');
fprintf('  %-12s %14s %16s %10s\n','pico','carrera [mm]','motor [N.m]','margen');
for F_obj = [6 4 3]
    d_nec = 2*Ec_mm/F_obj;                       % mm  carrera necesaria
    [~, T3o] = pares(F_obj, q, L, r_pal, m_esl, m_biela, g);
    fprintf('  %8.0f N   %14.1f %16.3f %10.1fx\n', F_obj, d_nec, T3o/N_red, T_nema/(T3o/N_red));
end
fprintf('  -> La amortiguacion debe bajar el pico de %.0f N a 4 N o menos\n', F_imp);
fprintf('     (unos 2 mm de carrera blanda en el fondo de la canasta).\n\n');

%% ---------------------- GRAFICOS ---------------------------------------
x    = linspace(0, L, 200)';        % mm  medido desde la punta (columna)
Mx_P = P*x;                        % contribucion de la carga puntual
Mx_q = q*x.^2/2;                   % contribucion del peso propio

figure('Name','Verificacion estructural','Color','w','Position',[100 100 980 400]);

subplot(1,2,1);
area(x, [Mx_P Mx_q], 'LineWidth', 1);
grid on; box on;
xlabel('distancia desde la punta  x [mm]');
ylabel('momento flector  M [N\cdotmm]');
title(sprintf('Momento flector   M_{max} = %.1f N\\cdotmm', M));
legend({sprintf('P\\cdotx  (%.1f)', M_P), sprintf('q\\cdotx^2/2  (%.1f)', M_q)}, ...
       'Location','northwest');

subplot(1,2,2);
F_obj = 4;
d_am  = 2*Ec_mm/F_obj;
plot([0 d_imp], [0 F_imp], 'LineWidth', 2); hold on;
plot([0 d_am ], [0 F_obj], 'LineWidth', 2);
yline(F_lim, '--', 'aca los motores pierden pasos', ...
      'LabelHorizontalAlignment','left');
grid on; box on;
xlabel('recorrido de frenado  d [mm]');
ylabel('fuerza  F [N]');
title(sprintf('Impacto: misma energia (%.2f mJ), distinto pico', Ec*1000));
legend({sprintf('sin amortiguar: %.1f N', F_imp), ...
        sprintf('amortiguado: %.0f N en %.1f mm', F_obj, d_am)}, ...
       'Location','northeast');

%% ---------------------- FUNCIONES --------------------------------------
function [T2, T3, F_biela] = pares(P, q, L, r_pal, m_esl, m_biela, g)
% Pares en las articulaciones para el brazo extendido horizontal.
% P       [N]    carga vertical aplicada en la punta del antebrazo
% Devuelve T2 (hombro) y T3 (palanca del paralelogramo) en N.m
    M_A     = q*L^2/2 + P*L;                 % N.mm  momento en el codo
    F_biela = M_A/r_pal;                     % N     biela = miembro de dos fuerzas
    T3      = M_A/1000;                      % N.m   T3 = F*r_pal = M_A
    R_A     = (m_esl/1000)*g + P;            % N     reaccion vertical en el codo
    M_O     = (m_esl/1000)*g*L/2 ...         % peso del brazo inferior
            + R_A*L ...                      % reaccion del antebrazo
            + (m_biela/1000)*g*L/2;          % peso de la biela
    T2      = M_O/1000 ;                     % N.m
end

function s = veredicto(ok)
    if ok, s = 'VERIFICA'; else, s = 'NO VERIFICA - revisar'; end
end

% diary off;
