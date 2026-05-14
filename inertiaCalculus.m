%% -------------------------------------------------------------------------
% CÁLCULO DE PROPIEDADES MÁSICAS: SISTEMA MULTICUERPO (12U + CARGAS)
% -------------------------------------------------------------------------
% DESCRIPCIÓN:
%   Calcula el Centro de Masa (CoM) y el Tensor de Inercia de un sistema
%   multicuerpo. El script está estructurado para ser modular, permitiendo
%   añadir o modificar cuerpos fácilmente.
%
% ESTRUCTURA:
%   1. CONFIGURACIÓN: Define parámetros globales y constantes.
%   2. DEFINICIÓN DE CUERPOS: Crea estructuras para cada componente.
%   3. CÁLCULO: Calcula propiedades del sistema total y subsistemas.
%   4. RESULTADOS: Muestra los datos en consola.
%   5. VISUALIZACIÓN: Genera un gráfico 3D de la configuración.
%
% UNIDADES: Sistema Internacional (kg, m, kg·m²)
% -------------------------------------------------------------------------

clear; clc; close all;

%% 1. CONFIGURACIÓN
% Parámetros de entrada y constantes
config = setupConfiguration();

%% 2. DEFINICIÓN DE CUERPOS
% Crear una celda de estructuras, donde cada estructura define un cuerpo.
bodies = defineBodies(config);

%% 3. CÁLCULO DE PROPIEDADES DEL SISTEMA
% Calcular CoM e Inercia para el sistema completo
system_total = calculateSystemProperties(bodies);

% Calcular propiedades para un subsistema (ej. solo cargas útiles)
payload_bodies = {bodies{2}, bodies{3}}; % Cuerpos B y C
payload_subsystem = calculateSystemProperties(payload_bodies);

%% 4. MOSTRAR RESULTADOS
displayResults('COMPLETE SYSTEM (A + B + C)', system_total);
displayResults('PAYLOAD SUBSYSTEM (B + C)', payload_subsystem);

% Generar código LaTeX para la tabla
printLatexTable(config, system_total, payload_subsystem);

%% 5. VISUALIZACIÓN 3D
plotConfiguration(bodies, system_total, payload_subsystem);

%% 6. ANÁLISIS ESTADÍSTICO (MONTE CARLO)
% Genera variaciones aleatorias para evaluar la incertidumbre del tensor
runMonteCarloInertia(config, bodies, 10000, 'COMPLETE SYSTEM (A + B + C)'); 
runMonteCarloInertia(config, payload_bodies, 10000, 'PAYLOAD SUBSYSTEM (B + C)');

%% -------------------------------------------------------------------------
%  --- FUNCIONES DE CONFIGURACIÓN Y CÁLCULO ---
%  -------------------------------------------------------------------------

function config = setupConfiguration()
    % Agrupa todos los parámetros de entrada y constantes en una estructura.
    config.error_margin = 0.30; % Porcentaje de error en masa
    config.G_MM2_TO_KG_M2 = 1e-9; % Conversión de g·mm² a kg·m²
    config.MM_TO_M = 1e-3;        % Conversión de mm a m
    
    % Desalineaciones estructurales nominales (offsets de montaje en X, Y, Z)
    config.misalignment_b = [2.0, -1.5, 0] * config.MM_TO_M; % SAT-B desviado 2mm en X, -1.5mm en Y
    config.misalignment_c = [-1.0, 2.0, 0] * config.MM_TO_M; % SAT-C desviado respecto a B
    config.assembly_tolerance = 2.0 * config.MM_TO_M;        % Tolerancia aleatoria máxima (3-sigma)
end

function bodies = defineBodies(config)
    % Define las propiedades de cada cuerpo del sistema.

    % --- Cuerpo A: Satélite 12U (Paralelepípedo) ---
    body_a.name = 'SAT-A (12U)';
    mass_a_base = 1.343382;
    body_a.mass = mass_a_base * (1 + config.error_margin);
    body_a.dimensions = [226.3, 226.3, 340.5] * config.MM_TO_M; % [lx, ly, lz]
    % CoM local respecto al centro geométrico del cuerpo
    body_a.com_offset = [0.05, -0.44, -149.03] * config.MM_TO_M;
    % Posición del centro geométrico del cuerpo en el marco global
    body_a.position = [0, 0, 0];
    % Tensor de inercia base (respecto a su propio CoM)
    I_a_gmm2 = [ 31082487.31,    -7248.29,     6857.64;
                 -7248.29,  31252296.38,   -66413.94;
                  6857.64,    -66413.94, 19986919.63 ];
    % Se asume que la inercia escala con la masa para mantener la consistencia.
    body_a.inertia_com = (I_a_gmm2 * config.G_MM2_TO_KG_M2) * (1 + config.error_margin);
    body_a.shape = 'cuboid';
    body_a.color = [0.2 0.4 0.8];

    % --- Cuerpo B: Carga Útil 1 (Esfera) ---
    body_b.name = 'SAT-B (Payload)';
    mass_b_base = 5.0;
    body_b.mass = mass_b_base * (1 + config.error_margin);
    body_b.dimensions = 0.1; % Radio
    body_b.com_offset = [0, 0, 0]; % Esfera homogénea, CoM en el centro
    % Posición del centro geométrico
    lz_a = body_a.dimensions(3);
    r_b = body_b.dimensions;
    ideal_pos_b = [0, 0, (67.3e-3 - lz_a/2 + r_b)];
    body_b.position = ideal_pos_b + config.misalignment_b;
    % Inercia (respecto a su propio CoM), calculada con la masa actualizada.
    body_b.inertia_com = calculateSphereInertia(body_b.mass, r_b, 'hollow');
    body_b.shape = 'sphere';
    body_b.color = [0.9 0.3 0.3];

    % --- Cuerpo C: Carga Útil 2 (Esfera Pequeña) ---
    body_c.name = 'SAT-C (Payload)';
    mass_c_base = 0.5;
    body_c.mass = mass_c_base * (1 + config.error_margin);
    body_c.dimensions = 0.05; % Radio
    body_c.com_offset = [0, 0, 0];
    % Posición del centro geométrico
    r_c = body_c.dimensions;
    ideal_pos_c = body_b.position + [0, 0, r_b + r_c];
    body_c.position = ideal_pos_c + config.misalignment_c;
    % Inercia (respecto a su propio CoM), calculada con la masa actualizada.
    body_c.inertia_com = calculateSphereInertia(body_c.mass, r_c, 'hollow');
    body_c.shape = 'sphere';
    body_c.color = [0.3 0.8 0.3];

    bodies = {body_a, body_b, body_c};
end

function I_com = calculateSphereInertia(mass, radius, type)
    % Calcula el tensor de inercia para una esfera (sólida o hueca).
    if strcmpi(type, 'hollow')
        factor = 2/3; % Esfera hueca
    elseif strcmpi(type, 'solid')
        factor = 2/5; % Esfera sólida
    else
        error('Tipo de esfera no válido. Use "solid" o "hollow".');
    end
    I_val = factor * mass * radius^2;
    I_com = diag([I_val, I_val, I_val]);
end

function system = calculateSystemProperties(bodies)
    % Calcula el CoM y el Tensor de Inercia para un conjunto de cuerpos.
    
    total_mass = 0;
    weighted_com_sum = [0; 0; 0];
    
    % 1. Calcular masa total y CoM del sistema
    for i = 1:length(bodies)
        body = bodies{i};
        total_mass = total_mass + body.mass;
        
        % Vector de posición absoluto del CoM de cada cuerpo
        body_com_abs = body.position(:) + body.com_offset(:);
        weighted_com_sum = weighted_com_sum + body.mass * body_com_abs;
    end
    
    system.mass = total_mass;
    system.com = weighted_com_sum / total_mass;
    
    % 2. Calcular tensor de inercia total usando Teorema de Steiner
    I_total_com = zeros(3, 3);
    
    % Función para el término del Teorema de Ejes Paralelos: m * ((r'r)I - rr')
    parallelAxisTerm = @(m, r) m * (dot(r,r)*eye(3) - (r(:)*r(:).'));

    for i = 1:length(bodies)
        body = bodies{i};
        
        % Vector desde el CoM del sistema al CoM del cuerpo
        r_vec = (body.position(:) + body.com_offset(:)) - system.com;
        
        % Sumar inercia local + término de transporte
        I_total_com = I_total_com + body.inertia_com + parallelAxisTerm(body.mass, r_vec);
    end
    
    system.inertia = I_total_com;
end

function displayResults(title_str, system)
    % Muestra los resultados calculados en la consola.
    fprintf('\n=================================================\n');
    fprintf(' RESULTADOS: %s\n', upper(title_str));
    fprintf('=================================================\n');
    fprintf('Masa Total:       %.4f kg\n', system.mass);
    fprintf('Centro de Masa:   [%.4f, %.4f, %.4f] m\n', system.com);
    fprintf('Tensor de Inercia (Respecto al CoM Total) [kg·m²]:\n');
    disp(system.inertia);
end

function printLatexTable(config, sys_abc, sys_bc)
    % Genera el código LaTeX para la tabla de propiedades y lo imprime en consola
    fprintf('\n%% --- CÓDIGO LATEX GENERADO AUTOMÁTICAMENTE ---\n');
    fprintf('\\begin{table}[ht]\n');
    fprintf('\\centering\n');
    fprintf('\\caption{Mass and inertial properties for the integrated system and payload subsystem, including structural misalignments and mass uncertainties.}\n');
    fprintf('\\label{tab:inertia_results}\n');
    fprintf('\\begin{tabular}{lcc}\n');
    fprintf('\\hline\n');
    fprintf('\\textbf{Parameter} & \\textbf{SAT-ABC} & \\textbf{SAT-BC} \\\\ \\hline\n');
    fprintf('Total Mass (kg) & %.4f & %.4f \\\\\n', sys_abc.mass, sys_bc.mass);
    fprintf('Center of Mass (m) & $[%.4f, %.*f, %.4f]$ & $[%.4f, %.*f, %.4f]$ \\\\ \\hline\n', ...
        sys_abc.com(1), 4, sys_abc.com(2), sys_abc.com(3), ...
        sys_bc.com(1), 4, sys_bc.com(2), sys_bc.com(3));
        
    fprintf('\\textbf{Uncertainties \\& Misalignments} & \\multicolumn{2}{c}{} \\\\ \\hline\n');
    fprintf('Mass Uncertainty Margin & \\multicolumn{2}{c}{%.1f\\%%} \\\\\n', config.error_margin * 100);
    fprintf('Assembly Tolerance (3$\\sigma$) & \\multicolumn{2}{c}{%.1f mm} \\\\\n', config.assembly_tolerance * 1000);
    fprintf('Nominal Offset SAT-B (mm) & \\multicolumn{2}{c}{$[%.1f, %.1f, %.1f]$} \\\\\n', ...
        config.misalignment_b(1)*1000, config.misalignment_b(2)*1000, config.misalignment_b(3)*1000);
    fprintf('Nominal Offset SAT-C (mm) & \\multicolumn{2}{c}{$[%.1f, %.1f, %.1f]$} \\\\ \\hline\n', ...
        config.misalignment_c(1)*1000, config.misalignment_c(2)*1000, config.misalignment_c(3)*1000);
        
    fprintf('\\textbf{Inertia Tensor ($J_{CoM}$)} & \\multicolumn{2}{c}{\\textbf{Components [kg$\\cdot$m$^2$]}} \\\\ \\hline\n');
    fprintf('$J_{xx}$ & %.4f & %.4f \\\\\n', sys_abc.inertia(1,1), sys_bc.inertia(1,1));
    fprintf('$J_{yy}$ & %.4f & %.4f \\\\\n', sys_abc.inertia(2,2), sys_bc.inertia(2,2));
    fprintf('$J_{zz}$ & %.4f & %.4f \\\\\n', sys_abc.inertia(3,3), sys_bc.inertia(3,3));
    fprintf('$J_{xy}$ & %.4f & %.4f \\\\\n', sys_abc.inertia(1,2), sys_bc.inertia(1,2));
    fprintf('$J_{xz}$ & %.4f & %.4f \\\\\n', sys_abc.inertia(1,3), sys_bc.inertia(1,3));
    fprintf('$J_{yz}$ & %.4f & %.4f \\\\ \\hline\n', sys_abc.inertia(2,3), sys_bc.inertia(2,3));
    fprintf('\\end{tabular}\n');
    fprintf('\\end{table}\n');
    fprintf('%% ---------------------------------------------\n\n');
end

function plotConfiguration(bodies, system_total, system_payload)
    % Dibuja la configuración 3D de los cuerpos y sus centros de masa.
    
    figure('Name','Multibody Configuration','Color','w'); 
    hold on; axis equal; grid on; view(40,25);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Spatial Configuration and Centers of Mass');

    [xs, ys, zs] = sphere(50); % Plantilla para esferas
    
    legend_handles = [];
    legend_labels = {};

    % Dibujar cada cuerpo
    for i = 1:length(bodies)
        body = bodies{i};
        
        if strcmp(body.shape, 'cuboid')
            hx = body.dimensions(1)/2; hy = body.dimensions(2)/2; hz = body.dimensions(3)/2;
            V = [-hx -hy -hz; hx -hy -hz; hx hy -hz; -hx hy -hz; 
                 -hx -hy  hz; hx -hy  hz; hx hy  hz; -hx hy  hz];
            F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
            
            patch('Vertices',V + body.position, 'Faces',F, 'FaceColor', body.color, ...
                  'FaceAlpha',0.15, 'EdgeColor', body.color*0.7, 'LineWidth',1.2);
        
        elseif strcmp(body.shape, 'sphere')
            r = body.dimensions;
            surf(r*xs + body.position(1), r*ys + body.position(2), r*zs + body.position(3), ...
                 'FaceColor', body.color, 'FaceAlpha',0.2, 'EdgeColor','none');
        end
        
        % Dibujar CoM individual
        com_abs = body.position + body.com_offset;
        h = plot3(com_abs(1), com_abs(2), com_abs(3), 'o', ...
                  'MarkerFaceColor', body.color, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
        legend_handles = [legend_handles, h];
        legend_labels{end+1} = ['CoM ' body.name];
    end

    % Dibujar CoM del sistema total
    h = plot3(system_total.com(1), system_total.com(2), system_total.com(3), ...
        'kp', 'MarkerFaceColor','y', 'MarkerSize',12, 'LineWidth',1.5);
    legend_handles = [legend_handles, h];
    legend_labels{end+1} = 'CoM TOTAL (SAT-ABC)';

    % Dibujar CoM del subsistema de cargas
    h = plot3(system_payload.com(1), system_payload.com(2), system_payload.com(3), ...
        'ms', 'MarkerFaceColor','m', 'MarkerSize',10, 'LineWidth',1.5);
    legend_handles = [legend_handles, h];
    legend_labels{end+1} = 'CoM Subsystem (SAT-BC)';

    legend(legend_handles, legend_labels, 'Location', 'northeastoutside', 'Interpreter', 'none');
    camlight headlight; lighting gouraud;
end

function runMonteCarloInertia(config, base_bodies, num_samples, plot_title)
    % Ejecuta un análisis de Monte Carlo variando las masas y posiciones de los
    % cuerpos basándose en una distribución normal para evaluar la incertidumbre
    % del Tensor de Inercia resultante.
    
    fprintf('\n=================================================\n');
    fprintf(' INICIANDO ANÁLISIS ESTADÍSTICO (MONTE CARLO)\n');
    fprintf(' Sistema: %s\n', plot_title);
    fprintf(' Muestras: %d\n', num_samples);
    fprintf('=================================================\n');
    
    Ixx_samples = zeros(num_samples, 1);
    Iyy_samples = zeros(num_samples, 1);
    Izz_samples = zeros(num_samples, 1);
    Ixy_samples = zeros(num_samples, 1);
    Ixz_samples = zeros(num_samples, 1);
    Iyz_samples = zeros(num_samples, 1);
    
    % Asumimos que el "margen de error" equivale a 3 desviaciones estándar (3 sigma)
    % para que el 99.7% de los valores caigan dentro de este margen.
    sigma_factor = config.error_margin / 3;
    
    for i = 1:num_samples
        % Copiar los cuerpos base (pueden ser 2, 3 o N cuerpos)
        bodies_mc = base_bodies;
        
        % Aplicar variación estadística a cada cuerpo
        for j = 1:length(bodies_mc)
            % Variación normal de la masa (media = masa base, std = masa_base * sigma)
            masa_base = bodies_mc{j}.mass / (1 + config.error_margin); % Revertir el error estático temporalmente
            std_mass = masa_base * sigma_factor;
            bodies_mc{j}.mass = masa_base + randn() * std_mass;
            
            if strcmp(bodies_mc{j}.shape, 'sphere')
               % Para las esferas, recalcular inercia con la masa variada
               bodies_mc{j}.inertia_com = calculateSphereInertia(bodies_mc{j}.mass, bodies_mc{j}.dimensions, 'hollow');
            else
               % Para otros cuerpos (ej. chasis), escalar inercia proporcionalmente a la nueva masa
               bodies_mc{j}.inertia_com = bodies_mc{j}.inertia_com / (1 + config.error_margin) * (bodies_mc{j}.mass / masa_base);
            end
            
            % Simular error aleatorio en el ensamblaje o posición de componentes internos.
            % Dividimos assembly_tolerance por 3 asumiendo que el valor en config es el límite 3-sigma
            % (99.7% de probabilidad de caer en esa tolerancia)
            std_tolerance = config.assembly_tolerance / 3;
            bodies_mc{j}.com_offset = bodies_mc{j}.com_offset + randn(1,3) * std_tolerance;
        end
        
        % Calcular propiedades del sistema con las variaciones
        system_mc = calculateSystemProperties(bodies_mc);
        
        Ixx_samples(i) = system_mc.inertia(1,1);
        Iyy_samples(i) = system_mc.inertia(2,2);
        Izz_samples(i) = system_mc.inertia(3,3);
        Ixy_samples(i) = system_mc.inertia(1,2);
        Ixz_samples(i) = system_mc.inertia(1,3);
        Iyz_samples(i) = system_mc.inertia(2,3);
    end
    
    % Mostrar distribuciones en una figura
    figure('Name', ['Monte Carlo: Inertia - ' plot_title], 'Color', 'w', 'Position', [100, 100, 1200, 600]);
    subplot(2,3,1); histogram(Ixx_samples, 30, 'FaceColor', '#0072BD'); title('I_{xx} Distribution'); xlabel('kg·m^2'); ylabel('Frequency'); grid on;
    subplot(2,3,2); histogram(Iyy_samples, 30, 'FaceColor', '#D95319'); title('I_{yy} Distribution'); xlabel('kg·m^2'); grid on;
    subplot(2,3,3); histogram(Izz_samples, 30, 'FaceColor', '#EDB120'); title('I_{zz} Distribution'); xlabel('kg·m^2'); grid on;
    
    subplot(2,3,4); histogram(Ixy_samples, 30, 'FaceColor', '#7E2F8E'); title('I_{xy} Distribution'); xlabel('kg·m^2'); ylabel('Frequency'); grid on;
    subplot(2,3,5); histogram(Ixz_samples, 30, 'FaceColor', '#77AC30'); title('I_{xz} Distribution'); xlabel('kg·m^2'); grid on;
    subplot(2,3,6); histogram(Iyz_samples, 30, 'FaceColor', '#4DBEEE'); title('I_{yz} Distribution'); xlabel('kg·m^2'); grid on;
    
    fprintf('Ixx -> Media: %.6e, Desv. Est: %.6e\n', mean(Ixx_samples), std(Ixx_samples));
    fprintf('Iyy -> Media: %.6e, Desv. Est: %.6e\n', mean(Iyy_samples), std(Iyy_samples));
    fprintf('Izz -> Media: %.6e, Desv. Est: %.6e\n', mean(Izz_samples), std(Izz_samples));
    fprintf('Ixy -> Media: %.6e, Desv. Est: %.6e\n', mean(Ixy_samples), std(Ixy_samples));
    fprintf('Ixz -> Media: %.6e, Desv. Est: %.6e\n', mean(Ixz_samples), std(Ixz_samples));
    fprintf('Iyz -> Media: %.6e, Desv. Est: %.6e\n', mean(Iyz_samples), std(Iyz_samples));
    
    % --- ANÁLISIS DE PEORES CASOS ---
    fprintf('\n--- WORST CASES (Absolute Extremes from %d samples) ---\n', num_samples);
    fprintf('Ixx -> Min: %.6e, Max: %.6e\n', min(Ixx_samples), max(Ixx_samples));
    fprintf('Iyy -> Min: %.6e, Max: %.6e\n', min(Iyy_samples), max(Iyy_samples));
    fprintf('Izz -> Min: %.6e, Max: %.6e\n', min(Izz_samples), max(Izz_samples));
    fprintf('Ixy -> Min: %.6e, Max: %.6e\n', min(Ixy_samples), max(Ixy_samples));
    fprintf('Ixz -> Min: %.6e, Max: %.6e\n', min(Ixz_samples), max(Ixz_samples));
    fprintf('Iyz -> Min: %.6e, Max: %.6e\n', min(Iyz_samples), max(Iyz_samples));
    
    % Extraer la matriz de inercia físicamente válida con el peor acoplamiento cruzado
    cross_coupling_magnitude = abs(Ixy_samples) + abs(Ixz_samples) + abs(Iyz_samples);
    [~, worst_idx] = max(cross_coupling_magnitude);
    
    fprintf('\n--- WORST CASE TENSOR (Maximum Cross-Coupling) ---\n');
    fprintf('Sample #%d produces the highest cross-coupling interference:\n', worst_idx);
    fprintf('  [ %.6e,  %.6e,  %.6e ]\n', Ixx_samples(worst_idx), Ixy_samples(worst_idx), Ixz_samples(worst_idx));
    fprintf('  [ %.6e,  %.6e,  %.6e ]\n', Ixy_samples(worst_idx), Iyy_samples(worst_idx), Iyz_samples(worst_idx));
    fprintf('  [ %.6e,  %.6e,  %.6e ]\n\n', Ixz_samples(worst_idx), Iyz_samples(worst_idx), Izz_samples(worst_idx));
end