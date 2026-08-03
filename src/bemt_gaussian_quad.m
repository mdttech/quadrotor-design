%% Aeromechanics of Unmanned Aerial Systems - Homework 4
% Unified Script for Q1 and Q2
% This algorithm uses Gaussian Quadrature Method

clear; close all; clc;

% Initialize the master figure for the SVG output (4x3 grid for 12 plots)
master_fig = figure('Name', 'HW4 Aeromechanics Results', 'Position', [50, 50, 1800, 1200], 'Color', 'w');
t = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'Helicopter Hover Aeromechanics (BEMT Analysis)', 'FontWeight', 'bold', 'FontSize', 18, 'Color', 'k');

% Initialize a cell array to collect all text outputs for the CSV
ResultsData = cell(0, 4); % Columns: Question, Metric, Case, Value

% Define High-Contrast Colors to prevent Dark Mode invisibility
line_colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E'}; % Blue, Orange, Yellow, Purple
line_styles = {'-','--','-.',':'};

%% ========================================================================
%% 1. PARAMETERS & GAUSS-6 SETUP
%% ========================================================================
p.rho      = 1.225;        % kg/m^3
p.Nb       = 2;            % number of blades
p.R        = 1.0;          % m (blade radius)
p.c_root   = 0.09;         % m (root chord)
p.Cd0      = 0.01;         % profile drag coefficient
p.Cl_alpha = 5.73;         % per radian
p.Omega    = 150;          % rad/s
p.T_req    = 300;          % Required thrust (N)
p.x_root   = 0.2;          % nondimensional root cutout (r/R)
p.M        = 10;           % number of blade elements

p.a_global = p.x_root * p.R;  
p.b_global = p.R;           
p.r_edges  = linspace(p.a_global, p.b_global, p.M+1);  

% Gauss Quadrature points and weights
p.y_gauss = [-0.932469514203152; -0.661209386466265; -0.238619186083197; ...
              0.238619186083197;  0.661209386466265;  0.932469514203152];
p.w_gauss = [ 0.171324492379170;  0.360761573048139;  0.467913934572691; ...
              0.467913934572691;  0.360761573048139;  0.171324492379170];

p.A = pi * p.R^2;
p.ref_scale_T = p.rho * p.A * (p.Omega * p.R)^2;   
p.ref_scale_Q = p.ref_scale_T * p.R;

% Function Handles for Chords
p.c_of_x = @(x) p.c_root * ones(size(x)); 
p.c_tip_factor = 0.5;
p.c_tip = p.c_root * p.c_tip_factor;
p.c_taper = @(x) ( p.c_root + (p.c_tip - p.c_root) .* ((x - p.x_root) ./ (1 - p.x_root)) );

% Function Handles for Twists
p.ideal_theta       = @(theta_tip, xvec) min( deg2rad(40), theta_tip ./ xvec );  
p.no_twist_theta    = @(theta_tip, xvec) theta_tip * ones(size(xvec));
total_twist_rad     = deg2rad(-15);
theta_root_offset   = -total_twist_rad;    
p.linear_twist_theta= @(theta_tip, xvec) ( theta_tip + theta_root_offset .* ( (xvec - 1) ./ (p.x_root - 1) ) );
p.ideal_taper_theta = @(theta_tip, xvec) p.ideal_theta(theta_tip, xvec); 

%% ========================================================================
%% Q1(a) -- Outer Loop for Reference Pitch & Pitch Distributions
%% ========================================================================
cases = {'ideal','no','linear','ideal_taper'};
theta_tip_solution = zeros(size(cases));
x_plot = linspace(p.x_root, 1, 300)';   
theta_profiles_deg = zeros(length(x_plot), length(cases));

opts = optimset('TolX',1e-6,'Display','off');
theta_lo = deg2rad(2);
theta_hi = deg2rad(40);

fprintf('Starting Q1(a) Root Finding...\n');
for idx = 1:length(cases)
    case_id = cases{idx};
    objfun = @(th) compute_total_thrust_theta(th, case_id, p) - p.T_req;
   
    try
        theta_sol = fzero(objfun, [theta_lo theta_hi], opts);
    catch
        thetas_grid = linspace(theta_lo, theta_hi, 200);
        errs = arrayfun(objfun, thetas_grid);
        [~, ind] = min(abs(errs));
        theta_sol = fminsearch(@(t) abs(objfun(t)), thetas_grid(ind), opts);
    end

    theta_tip_solution(idx) = theta_sol;
    achieved_thrust = compute_total_thrust_theta(theta_sol, case_id, p);
    
    % Store in CSV Data
    ResultsData(end+1,:) = {'Q1(a)', 'Theta Tip (deg)', case_id, rad2deg(theta_sol)};
    ResultsData(end+1,:) = {'Q1(a)', 'Achieved Thrust (N)', case_id, achieved_thrust};

    % Generate Theta Profiles
    switch case_id
        case 'ideal',       theta_vec = p.ideal_theta(theta_sol, x_plot);
        case 'no',          theta_vec = p.no_twist_theta(theta_sol, x_plot);
        case 'linear',      theta_vec = p.linear_twist_theta(theta_sol, x_plot);
        case 'ideal_taper', theta_vec = p.ideal_taper_theta(theta_sol, x_plot);
    end
    theta_profiles_deg(:, idx) = rad2deg(theta_vec(:));
end

% Plot 1: Theta vs x
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
legend_entries = cell(1,length(cases));
for k = 1:length(cases)
    plot(x_plot, theta_profiles_deg(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
    legend_entries{k} = sprintf('%s (\\theta_{tip}=%.2f^\\circ)', cases{k}, rad2deg(theta_tip_solution(k)));
end
xlabel('Nondimensional radius x = r/R', 'Color', 'k'); ylabel('\theta(x) [deg]', 'Color', 'k'); title('Q1(a): Pitch distribution \theta(x)', 'Color', 'k');
legend(legend_entries, 'Location', 'northeast', 'Box', 'off', 'TextColor', 'k');

% Compare Numerical vs Closed Form for Ideal Twist
k_ideal = find(strcmp(cases,'ideal'),1);
theta_tip_ideal = theta_tip_solution(k_ideal);
sigma_const = p.Nb * p.c_root / (pi * p.R);   
lambda_num = zeros(length(x_plot),1);

for i = 1:length(x_plot)
    xi = x_plot(i);
    th_i = p.ideal_theta(theta_tip_ideal, xi);
    A = 4 * xi; B = 0.5 * sigma_const * p.Cl_alpha * xi; C = 0.5 * sigma_const * p.Cl_alpha * xi^2 * th_i;
    discr = max(B^2 + 4*A*C, 0);
    lambda_num(i) = (-B + sqrt(discr)) / (2*A);
end

theta_local_ideal = p.ideal_theta(theta_tip_ideal, x_plot);
lambda_closed = (sigma_const * p.Cl_alpha / 16) .* ( sqrt( 1 + (32 .* (theta_local_ideal .* x_plot)) ./ (sigma_const * p.Cl_alpha) ) - 1 );

% Plot 2: Numerical vs Closed Form
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x_plot, lambda_num, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 1.5, 'DisplayName', 'Numerical \lambda(x)');
plot(x_plot, lambda_closed, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Closed-form \lambda_{closed}(x)');
xlabel('x = r/R', 'Color', 'k'); ylabel('Induced inflow \lambda(x)', 'Color', 'k'); title('Q1(a): Ideal Twist \lambda(x) Comparison', 'Color', 'k');
legend('Location','northeast','Box','off', 'TextColor', 'k');

% Plot 3: Absolute Difference
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x_plot, lambda_num - lambda_closed, 'Color', line_colors{3}, 'LineWidth', 1.5);
xlabel('x = r/R', 'Color', 'k'); ylabel('\lambda_{num} - \lambda_{closed}', 'Color', 'k'); title('Q1(a): Absolute Difference', 'Color', 'k');

%% ========================================================================
%% Q1(b) -- Variation of angle of attack alpha(x)
%% ========================================================================
alpha_profiles_deg = zeros(length(x_plot), length(cases));

for k = 1:length(cases)
    case_id = cases{k};
    theta_tip = theta_tip_solution(k);   
    
    switch case_id
        case 'ideal',       theta_vec = p.ideal_theta(theta_tip, x_plot);       c_vec = p.c_of_x(x_plot);
        case 'no',          theta_vec = p.no_twist_theta(theta_tip, x_plot);    c_vec = p.c_of_x(x_plot);
        case 'linear',      theta_vec = p.linear_twist_theta(theta_tip, x_plot);c_vec = p.c_of_x(x_plot);
        case 'ideal_taper', theta_vec = p.ideal_taper_theta(theta_tip, x_plot); c_vec = p.c_taper(x_plot);
    end

    sigma_vec = p.Nb .* c_vec ./ (pi * p.R);
    A = 4 .* x_plot; B = 0.5 .* sigma_vec .* p.Cl_alpha .* x_plot; C = 0.5 .* sigma_vec .* p.Cl_alpha .* (x_plot.^2) .* theta_vec;
    discr = max(B.^2 + 4 .* A .* C, 0);
    lambda_vec = max(( -B + sqrt(discr) ) ./ (2 .* A), 0);
    
    alpha_profiles_deg(:,k) = rad2deg(theta_vec - (lambda_vec ./ x_plot));
    
    ResultsData(end+1,:) = {'Q1(b)', 'Max Alpha (deg)', case_id, max(alpha_profiles_deg(:,k))};
    ResultsData(end+1,:) = {'Q1(b)', 'Min Alpha (deg)', case_id, min(alpha_profiles_deg(:,k))};
end

% Plot 4: Angle of Attack
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:length(cases)
    plot(x_plot, alpha_profiles_deg(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
end
xlabel('x = r/R', 'Color', 'k'); ylabel('\alpha(x) [deg]', 'Color', 'k'); title('Q1(b): Angle of attack \alpha(x)', 'Color', 'k');
legend(legend_entries, 'Location', 'northeast', 'Box', 'off', 'TextColor', 'k');

%% ========================================================================
%% Q1(c) -- Variation of induced inflow lambda(x)
%% ========================================================================
lambda_profiles = zeros(length(x_plot), length(cases));
[~, idx75] = min(abs(x_plot - 0.75));

for k = 1:length(cases)
    case_id = cases{k};
    theta_tip = theta_tip_solution(k);
    
    switch case_id
        case 'ideal',       theta_vec = p.ideal_theta(theta_tip, x_plot);       c_vec = p.c_of_x(x_plot);
        case 'no',          theta_vec = p.no_twist_theta(theta_tip, x_plot);    c_vec = p.c_of_x(x_plot);
        case 'linear',      theta_vec = p.linear_twist_theta(theta_tip, x_plot);c_vec = p.c_of_x(x_plot);
        case 'ideal_taper', theta_vec = p.ideal_taper_theta(theta_tip, x_plot); c_vec = p.c_taper(x_plot);
    end

    sigma_vec = p.Nb .* c_vec ./ (pi .* p.R);
    A = 4 .* x_plot; B = 0.5 .* sigma_vec .* p.Cl_alpha .* x_plot; C = 0.5 .* sigma_vec .* p.Cl_alpha .* (x_plot.^2) .* theta_vec;
    discr = max(B.^2 + 4 .* A .* C, 0);
    lambda_profiles(:,k) = max(( -B + sqrt(discr) ) ./ (2 .* A), 0);
    
    ResultsData(end+1,:) = {'Q1(c)', 'Lambda at 0.75', case_id, lambda_profiles(idx75,k)};
end

% Plot 5: Induced Inflow
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:length(cases)
    plot(x_plot, lambda_profiles(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
end
xlabel('x = r/R', 'Color', 'k'); ylabel('\lambda(x)', 'Color', 'k'); title('Q1(c): Induced inflow \lambda(x)', 'Color', 'k');
legend(legend_entries, 'Location', 'northeast', 'Box', 'off', 'TextColor', 'k');

%% ========================================================================
%% Q1(d) -- Thrust & Torque distributions
%% ========================================================================
dCT_dr_profiles  = zeros(length(x_plot), length(cases));   
dCQp_dr_profiles = zeros(length(x_plot), length(cases)); 
dCQi_dr_profiles = zeros(length(x_plot), length(cases));  
dCQ_dr_profiles  = zeros(length(x_plot), length(cases)); 

for k = 1:length(cases)
    case_id = cases{k};
    theta_tip = theta_tip_solution(k);
    
    switch case_id
        case 'ideal',       theta_vec = p.ideal_theta(theta_tip, x_plot);       c_vec = p.c_of_x(x_plot);
        case 'no',          theta_vec = p.no_twist_theta(theta_tip, x_plot);    c_vec = p.c_of_x(x_plot);
        case 'linear',      theta_vec = p.linear_twist_theta(theta_tip, x_plot);c_vec = p.c_of_x(x_plot);
        case 'ideal_taper', theta_vec = p.ideal_taper_theta(theta_tip, x_plot); c_vec = p.c_taper(x_plot);
    end

    r_vec = x_plot .* p.R;
    sigma_vec = p.Nb .* c_vec ./ (pi .* p.R);
    A = 4 .* x_plot; B = 0.5 .* sigma_vec .* p.Cl_alpha .* x_plot; C = 0.5 .* sigma_vec .* p.Cl_alpha .* (x_plot.^2) .* theta_vec;
    discr = max(B.^2 + 4 .* A .* C, 0); 
    lambda_vec = max(( -B + sqrt(discr) ) ./ (2 .* A), 0);
    Cl_vec = p.Cl_alpha .* ( theta_vec - (lambda_vec ./ x_plot) );

    f_r = 0.5 .* p.rho .* p.Nb .* (p.Omega .* r_vec).^2 .* c_vec .* Cl_vec;   
    dCT_dr_profiles(:,k) = f_r ./ p.ref_scale_T;

    dD_dr = 0.5 .* p.rho .* p.Nb .* (p.Omega .* r_vec).^2 .* c_vec .* p.Cd0;   
    dCQp_dr_profiles(:,k) = (r_vec .* dD_dr) ./ p.ref_scale_Q;  
    dCQi_dr_profiles(:,k) = (f_r .* lambda_vec .* p.R) ./ p.ref_scale_Q;   
    dCQ_dr_profiles(:,k)  = dCQp_dr_profiles(:,k) + dCQi_dr_profiles(:,k);
end

% Plot 6: dCT/dr
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:length(cases)
    plot(x_plot, dCT_dr_profiles(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
end
xlabel('x = r/R', 'Color', 'k'); ylabel('dC_T/dr', 'Color', 'k'); title('Q1(d): Nondim Thrust Density', 'Color', 'k');

% Plot 7: dCQ/dr
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:length(cases)
    plot(x_plot, dCQ_dr_profiles(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
end
xlabel('x = r/R', 'Color', 'k'); ylabel('dC_Q/dr', 'Color', 'k'); title('Q1(d): Total Torque Density', 'Color', 'k');

% Plot 8: dCQi/dr
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:length(cases)
    plot(x_plot, dCQi_dr_profiles(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
end
xlabel('x = r/R', 'Color', 'k'); ylabel('dC_{Qi}/dr', 'Color', 'k'); title('Q1(d): Induced Torque Density', 'Color', 'k');

% Plot 9: dCQp/dr
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:length(cases)
    plot(x_plot, dCQp_dr_profiles(:,k), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 2);
end
xlabel('x = r/R', 'Color', 'k'); ylabel('dC_{Qp}/dr', 'Color', 'k'); title('Q1(d): Profile Torque Density', 'Color', 'k');

%% ========================================================================
%% Q2 -- Prandtl tip-loss for "no" twist case
%% ========================================================================
theta_no_Q1 = theta_tip_solution(find(strcmp(cases,'no'),1));

objfun_tiploss = @(th) compute_total_thrust_theta_tiploss(th, 'no', p) - p.T_req;
try
    theta75_tiploss = fzero(objfun_tiploss, [deg2rad(0.5) deg2rad(60)], opts);
catch
    thetas_grid = linspace(deg2rad(0.5), deg2rad(60), 300);
    errs = arrayfun(objfun_tiploss, thetas_grid);
    [~, ind] = min(abs(errs));
    theta75_tiploss = fminsearch(@(t) abs(objfun_tiploss(t)), thetas_grid(ind), opts);
end

ResultsData(end+1,:) = {'Q2', 'Theta75 (no twist) with Tip Loss (deg)', 'no_twist_tiploss', rad2deg(theta75_tiploss)};
ResultsData(end+1,:) = {'Q2', 'Diff from Q1 no-twist (deg)', 'comparison', rad2deg(theta75_tiploss - theta_no_Q1)};

lambda_tiploss = zeros(length(x_plot),1);
dCT_dr_tiploss = zeros(length(x_plot),1);
dCQ_dr_tiploss = zeros(length(x_plot),1);

for i = 1:length(x_plot)
    xi = x_plot(i);
    sigma_i = p.Nb * p.c_root / (pi * p.R);
    lam_tl = lambda_with_tiploss_point_scalar(theta75_tiploss, xi, sigma_i, p.Nb, p.Cl_alpha);
    lambda_tiploss(i) = max(0, lam_tl);

    r_i = xi * p.R;
    Cl_i_tl = p.Cl_alpha * (theta75_tiploss - (lambda_tiploss(i)/xi));
    f_r_tl = 0.5 * p.rho * p.Nb * (p.Omega * r_i)^2 * p.c_root * Cl_i_tl;
    dCT_dr_tiploss(i) = f_r_tl / p.ref_scale_T;

    dD_dr_tl = 0.5 * p.rho * p.Nb * (p.Omega * r_i)^2 * p.c_root * p.Cd0;
    dCQ_dr_tiploss(i) = ((r_i * dD_dr_tl) + (f_r_tl * lambda_tiploss(i) * p.R)) / p.ref_scale_Q;
end

% Extract Q1 "no twist" curves for comparison
idx_no = find(strcmp(cases,'no'),1);
lambda_no = lambda_profiles(:, idx_no);
dCT_dr_no = dCT_dr_profiles(:, idx_no);
dCQ_dr_no = dCQ_dr_profiles(:, idx_no);

% Plot 10: Q2 Induced Inflow
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x_plot, lambda_no, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 2);
plot(x_plot, lambda_tiploss, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 2);
xlabel('x = r/R', 'Color', 'k'); ylabel('\lambda(x)', 'Color', 'k'); title('Q2: Tip-loss Effect on Inflow \lambda(x)', 'Color', 'k');
legend('No tip-loss', 'With tip-loss', 'Location','northeast','Box','off', 'TextColor', 'k');

% Plot 11: Q2 Thrust Distribution
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x_plot, dCT_dr_no, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 2);
plot(x_plot, dCT_dr_tiploss, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 2);
xlabel('x = r/R', 'Color', 'k'); ylabel('dC_T/dr', 'Color', 'k'); title('Q2: Tip-loss Effect on Thrust', 'Color', 'k');

% Plot 12: Q2 Torque Distribution
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x_plot, dCQ_dr_no, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 2);
plot(x_plot, dCQ_dr_tiploss, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 2);
xlabel('x = r/R', 'Color', 'k'); ylabel('dC_Q/dr', 'Color', 'k'); title('Q2: Tip-loss Effect on Torque', 'Color', 'k');

%% ========================================================================
%% FINAL EXPORTS: Save SVG and CSV
%% ========================================================================

% 1. Save all plots into a single SVG file
exportgraphics(master_fig, 'Homework4_Figures.svg', 'BackgroundColor', 'white', 'ContentType', 'vector');
disp('Successfully saved all 12 plots to: Homework4_Figures.svg');

% 2. Save all scalar output metrics into a single CSV file
ResultsTable = cell2table(ResultsData, 'VariableNames', {'Question', 'Metric', 'Case', 'Value'});
writetable(ResultsTable, 'Homework4_Data.csv');
disp('Successfully saved summary data to: Homework4_Data.csv');

%% ========================================================================
%% LOCAL FUNCTIONS (Must remain at the bottom of the script)
%% ========================================================================

% -------------------------------------------------------------------------
% Inner Loop without tip-loss (Q1)
% -------------------------------------------------------------------------
function T_total = compute_total_thrust_theta(theta_tip, case_id, p)
    T_total = 0.0;
    for k = 1:p.M
        a_k = p.r_edges(k);
        b_k = p.r_edges(k+1);
        elem_sum = 0.0;          
        half_len = (b_k - a_k) / 2;
        mid_pt = (b_k + a_k) / 2;

        for j = 1:6
            r_j = half_len * p.y_gauss(j) + mid_pt;        
            x_j = r_j / p.R;                    

            switch case_id
                case 'ideal',       theta_j = p.ideal_theta(theta_tip, x_j);       c_j = p.c_of_x(x_j);
                case 'no',          theta_j = p.no_twist_theta(theta_tip, x_j);    c_j = p.c_of_x(x_j);
                case 'linear',      theta_j = p.linear_twist_theta(theta_tip, x_j);c_j = p.c_of_x(x_j);
                case 'ideal_taper', theta_j = p.ideal_taper_theta(theta_tip, x_j); c_j = p.c_taper(x_j);
            end
            theta_j = theta_j(1); c_j = c_j(1);

            sigma_j = p.Nb * c_j / (pi * p.R);
            A = 4 * x_j;
            B = 0.5 * sigma_j * p.Cl_alpha * x_j;
            C = 0.5 * sigma_j * p.Cl_alpha * x_j^2 * theta_j;

            discr = B^2 + 4*A*C;
            if discr < 0, lambda_j = 0; else
                lambda_j = ( -B + sqrt(discr) ) / (2*A);
                if lambda_j < 0, lambda_j = ( -B - sqrt(discr) ) / (2*A); end
            end

            Cl_j = p.Cl_alpha * ( theta_j - (lambda_j / x_j) );
            f_rj = 0.5 * p.rho * p.Nb * (p.Omega * r_j)^2 * c_j * Cl_j;
            elem_sum = elem_sum + p.w_gauss(j) * f_rj;
        end
        T_total = T_total + (elem_sum * half_len);
    end
end

% -------------------------------------------------------------------------
% Iterative Lambda solver with Prandtl tip-loss (Q2)
% -------------------------------------------------------------------------
function lambda_ret = lambda_with_tiploss_point_scalar(theta_local, x_local, sigma_local, Nb_local, Cl_alpha_local)
    tol = 1e-8;
    maxit = 200;
    denom = sigma_local * Cl_alpha_local;
    if denom <= 0, lambda_old = 0; else
        arg0 = max(1 + (32 .* (theta_local .* x_local)) ./ denom, 1.0);
        lambda_old = (denom/16) * ( sqrt(arg0) - 1 );
    end

    for it = 1:maxit
        if x_local <= 0, phi = pi/2; else, phi = atan2(lambda_old, x_local); end
        sphi = sin(phi);
        if abs(sphi) < 1e-10, fval = Inf; else, fval = (Nb_local/2) * ( (1 - x_local) / ( x_local * sphi ) ); end

        if isinf(fval) && fval>0, expnegf = 0; else, expnegf = min(max(exp(-fval), 0), 1); end
        F = (2/pi) * acos(expnegf);

        if denom <= 0, lambda_new = 0; else
            arg2 = max(1 + (32 * F .* (theta_local .* x_local)) ./ denom, 1.0);
            lambda_new = (denom/16) * ( sqrt(arg2) - 1 );
        end

        if abs(lambda_new - lambda_old) < tol
            lambda_ret = lambda_new;
            return;
        end
        lambda_old = lambda_new;
    end
    lambda_ret = lambda_old;
end

% -------------------------------------------------------------------------
% Inner Loop with tip-loss (Q2)
% -------------------------------------------------------------------------
function T_total = compute_total_thrust_theta_tiploss(theta_ref, case_id, p)
    T_total = 0.0;
    for k = 1:p.M
        a_k = p.r_edges(k);
        b_k = p.r_edges(k+1);
        elem_sum = 0.0;
        half_len = (b_k - a_k) / 2;
        mid_pt = (b_k + a_k) / 2;

        for j = 1:length(p.y_gauss)
            r_j = half_len * p.y_gauss(j) + mid_pt;
            x_j = r_j / p.R;
            
            theta_j = theta_ref;
            c_j = p.c_root;
            sigma_j = p.Nb * c_j / (pi * p.R);
            
            lambda_j = lambda_with_tiploss_point_scalar(theta_j, x_j, sigma_j, p.Nb, p.Cl_alpha);
            Cl_j = p.Cl_alpha * ( theta_j - (lambda_j / x_j) );
            f_rj = 0.5 * p.rho * p.Nb * (p.Omega * r_j)^2 * c_j * Cl_j;
            elem_sum = elem_sum + p.w_gauss(j) * f_rj;
        end
        T_total = T_total + (elem_sum * half_len);
    end
end