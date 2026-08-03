%% Unified Script for BEMT Analysis (Q1 & Q2)
% % This algorithm uses Trapezoidal Method


clear; close all; clc;

% Initialize the master figure for the SVG output (3x3 grid for 9 plots)
master_fig = figure('Name', 'BEMT All Results', 'Position', [50, 50, 1600, 1200], 'Color', 'w');
t = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'BEMT Analysis Results (Q1 & Q2)', 'FontWeight', 'bold', 'FontSize', 16, 'Color', 'k');

% Define High-Contrast Colors to prevent Dark Mode invisibility
line_colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E'}; % Blue, Orange, Yellow, Purple
line_styles = {'-','--','-.',':'};

% Initialize a cell array to collect all text outputs for the CSV
ResultsData = cell(0, 4); % Columns: Question, Metric, Case, Value

%% ========================================================================
%% Q1(a) -- BEMT (trapezoidal inner loop) + root-find for theta75
%% ========================================================================
rho      = 1.225;        
Nb       = 2;            
R        = 1.0;          
c_root   = 0.09;         
C_l_alpha = 5.73;        
Omega    = 150;          
T_target = 300;          
x_root   = 0.2;          
N = 400;                 

x = linspace(x_root, 1, N);     
r = x * R;

cases = 1:4;
theta75_found_a = zeros(size(cases));
options = optimset('TolX',1e-6,'Display','off');
case_names = {'Ideal twist', 'No twist', 'Linear -15 deg', 'Ideal + taper'};

for cid = cases
    fun = @(theta75_deg) total_thrust_for_theta75(theta75_deg, cid, x, r, R, c_root, Nb, rho, Omega, C_l_alpha) - T_target;
    a = 0; b = 40; guess = 8;
    if fun(a) * fun(b) > 0, a = -10; b = 60; end
    try theta75_sol = fzero(fun, guess, options); catch, theta75_sol = fzero(fun, [a b], options); end
    theta75_found_a(cid) = theta75_sol;
    
    % Save to CSV Data
    achieved_thrust = total_thrust_for_theta75(theta75_sol, cid, x, r, R, c_root, Nb, rho, Omega, C_l_alpha);
    ResultsData(end+1,:) = {'Q1(a)', 'Theta75 (deg)', case_names{cid}, theta75_sol};
    ResultsData(end+1,:) = {'Q1(a)', 'Achieved Thrust (N)', case_names{cid}, achieved_thrust};
end

theta_deg_cases = zeros(numel(cases), numel(x)); 
for cid = cases
    theta75_deg = theta75_found_a(cid);
    switch cid
        case 1, theta_deg_cases(cid,:) = theta75_deg * (0.75 ./ x);
        case 2, theta_deg_cases(cid,:) = theta75_deg * ones(size(x));
        case 3
            m = (deg2rad(theta75_deg - 15) - deg2rad(theta75_deg)) / (1 - 0.75);
            theta_deg_cases(cid,:) = rad2deg(m .* x + (deg2rad(theta75_deg) - m * 0.75));
        case 4, theta_deg_cases(cid,:) = theta75_deg * (0.75 ./ x);
    end
end

% Plot 1
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:4
    plot(x, theta_deg_cases(k,:), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 1.6);
end
xlabel('r/R', 'Color', 'k'); ylabel('\theta (deg)', 'Color', 'k'); title('Q1(a): Pitch angle vs r/R', 'Color', 'k');
legend(case_names, 'Location','best', 'TextColor', 'k', 'Box', 'off');

% Plot 2
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x, theta_deg_cases(1,:), 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 1.8);
plot(x, theta75_found_a(1) * (0.75 ./ x), 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 1.4);
xlabel('r/R', 'Color', 'k'); ylabel('\theta (deg)', 'Color', 'k'); title('Q1(a): Ideal twist vs Closed-form', 'Color', 'k');
legend({'Numerical','Closed-form'}, 'Location','best', 'TextColor', 'k', 'Box', 'off');


%% ========================================================================
%% Q1(b) -- Angle of attack distribution
%% ========================================================================
alpha_deg_cases = zeros(numel(cases), numel(x)); 
for cid = cases
    theta75_deg = theta75_found_a(cid); % Reusing theta from part a
    switch cid
        case 1, theta_x_rad = deg2rad(theta75_deg) .* (0.75 ./ x); c_x = c_root * ones(size(x));
        case 2, theta_x_rad = deg2rad(theta75_deg) * ones(size(x)); c_x = c_root * ones(size(x));
        case 3
            m = (deg2rad(theta75_deg - 15) - deg2rad(theta75_deg)) / (1 - 0.75);
            theta_x_rad = m .* x + (deg2rad(theta75_deg) - m * 0.75); c_x = c_root * ones(size(x));
        case 4
            theta_x_rad = deg2rad(theta75_deg) .* (0.75 ./ x);
            c_x = c_root + (0.5 * c_root - c_root) .* ((x - x(1)) ./ (1 - x(1)));
    end

    sigma_x = Nb .* c_x ./ (pi * R);
    lambda_x = zeros(size(x));
    for i = 1:numel(x)
        A = 4 * x(i); B = 0.5 * sigma_x(i) * C_l_alpha * x(i); C = 0.5 * sigma_x(i) * C_l_alpha * x(i)^2 * theta_x_rad(i);
        disc = max(B^2 + 4 * A * C, 0);
        lambda_x(i) = (-B + sqrt(disc)) / (2 * A);
    end
    alpha_deg_cases(cid, :) = rad2deg(theta_x_rad - lambda_x ./ x);
    
    % Save to CSV Data
    ResultsData(end+1,:) = {'Q1(b)', 'Max Alpha (deg)', case_names{cid}, max(alpha_deg_cases(cid,:))};
    ResultsData(end+1,:) = {'Q1(b)', 'Min Alpha (deg)', case_names{cid}, min(alpha_deg_cases(cid,:))};
end

% Plot 3
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:4
    plot(x, alpha_deg_cases(k,:), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 1.6);
end
xlabel('r/R', 'Color', 'k'); ylabel('\alpha (deg)', 'Color', 'k'); title('Q1(b): Angle of attack vs r/R', 'Color', 'k');
legend(case_names, 'Location','best', 'TextColor', 'k', 'Box', 'off');

%% ========================================================================
%% Q1(c) -- Induced inflow distribution
%% ========================================================================
lambda_cases = zeros(numel(cases), numel(x)); 
[~, idx75] = min(abs(x - 0.75));

for cid = cases
    theta75_deg = theta75_found_a(cid);
    switch cid
        case 1, theta_x_rad = deg2rad(theta75_deg) .* (0.75 ./ x); c_x = c_root * ones(size(x));
        case 2, theta_x_rad = deg2rad(theta75_deg) * ones(size(x)); c_x = c_root * ones(size(x));
        case 3
            m = (deg2rad(theta75_deg - 15) - deg2rad(theta75_deg)) / (1 - 0.75);
            theta_x_rad = m .* x + (deg2rad(theta75_deg) - m * 0.75); c_x = c_root * ones(size(x));
        case 4
            theta_x_rad = deg2rad(theta75_deg) .* (0.75 ./ x);
            c_x = c_root + (0.5 * c_root - c_root) .* ((x - x(1)) ./ (1 - x(1)));
    end

    sigma_x = Nb .* c_x ./ (pi * R);
    for i = 1:numel(x)
        A = 4 * x(i); B = 0.5 * sigma_x(i) * C_l_alpha * x(i); C = 0.5 * sigma_x(i) * C_l_alpha * x(i)^2 * theta_x_rad(i);
        disc = max(B^2 + 4 * A * C, 0);
        lambda_cases(cid, i) = (-B + sqrt(disc)) / (2 * A);
    end
    
    % Save to CSV Data
    ResultsData(end+1,:) = {'Q1(c)', 'Lambda at 0.75', case_names{cid}, lambda_cases(cid, idx75)};
    ResultsData(end+1,:) = {'Q1(c)', 'Max Lambda', case_names{cid}, max(lambda_cases(cid,:))};
    ResultsData(end+1,:) = {'Q1(c)', 'Min Lambda', case_names{cid}, min(lambda_cases(cid,:))};
end

% Plot 4
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:4
    plot(x, lambda_cases(k,:), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 1.6);
end
xlabel('r/R', 'Color', 'k'); ylabel('\lambda', 'Color', 'k'); title('Q1(c): Induced inflow vs r/R', 'Color', 'k');
legend(case_names, 'Location','best', 'TextColor', 'k', 'Box', 'off');


%% ========================================================================
%% Q1(d) -- Thrust & Torque distributions
%% ========================================================================
Cdo = 0.01;         
A_area = pi * R^2;        
denom_CT = rho * A_area * (Omega * R)^2;            
denom_CQ = rho * A_area * (Omega * R)^2 * R;        

dTdr_cases = zeros(numel(cases), numel(x));    
dQpdr_cases = zeros(numel(cases), numel(x));   
dQidr_cases = zeros(numel(cases), numel(x));   

for cid = cases
    theta75_deg = theta75_found_a(cid);
    switch cid
        case 1, theta_x_rad = deg2rad(theta75_deg) .* (0.75 ./ x); c_x = c_root * ones(size(x));
        case 2, theta_x_rad = deg2rad(theta75_deg) * ones(size(x)); c_x = c_root * ones(size(x));
        case 3
            m = (deg2rad(theta75_deg - 15) - deg2rad(theta75_deg)) / (1 - 0.75);
            theta_x_rad = m .* x + (deg2rad(theta75_deg) - m * 0.75); c_x = c_root * ones(size(x));
        case 4
            theta_x_rad = deg2rad(theta75_deg) .* (0.75 ./ x);
            c_x = c_root + (0.5 * c_root - c_root) .* ((x - x(1)) ./ (1 - x(1)));
    end

    sigma_x = Nb .* c_x ./ (pi * R);
    for i = 1:numel(x)
        A = 4 * x(i); B = 0.5 * sigma_x(i) * C_l_alpha * x(i); C = 0.5 * sigma_x(i) * C_l_alpha * x(i)^2 * theta_x_rad(i);
        disc = max(B^2 + 4 * A * C, 0);
        lambda_i = (-B + sqrt(disc)) / (2 * A);
        
        Cl_i = C_l_alpha * (theta_x_rad(i) - lambda_i / x(i));
        U2 = (Omega * r(i))^2;
        dTdr_cases(cid,i) = 0.5 * rho * Nb * U2 * Cl_i * c_x(i);   
        dQpdr_cases(cid,i) = r(i) * (0.5 * rho * Nb * U2 * c_x(i) * Cdo);  
        dQidr_cases(cid,i) = dTdr_cases(cid,i) * lambda_i * R;  
    end
end

dCTdr_cases = dTdr_cases ./ denom_CT;          
CT_cum_cases = cumtrapz(r, dCTdr_cases, 2);      
Cq_cum_cases = cumtrapz(r, (dQpdr_cases + dQidr_cases) ./ denom_CQ, 2);
Cqp_cum_cases = cumtrapz(r, dQpdr_cases ./ denom_CQ, 2);
Cqi_cum_cases = cumtrapz(r, dQidr_cases ./ denom_CQ, 2);

for cid = cases
    ResultsData(end+1,:) = {'Q1(d)', 'Total C_T', case_names{cid}, CT_cum_cases(cid,end)};
    ResultsData(end+1,:) = {'Q1(d)', 'Total C_Q', case_names{cid}, Cq_cum_cases(cid,end)};
    ResultsData(end+1,:) = {'Q1(d)', 'Profile C_Qp', case_names{cid}, Cqp_cum_cases(cid,end)};
    ResultsData(end+1,:) = {'Q1(d)', 'Induced C_Qi', case_names{cid}, Cqi_cum_cases(cid,end)};
end

% Plot 5
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:4
    plot(x, dCTdr_cases(k,:), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 1.6);
end
xlabel('r/R', 'Color', 'k'); ylabel('dC_T/dr', 'Color', 'k'); title('Q1(d): Thrust coefficient density', 'Color', 'k');
legend(case_names, 'Location','best', 'TextColor', 'k', 'Box', 'off');

% Plot 6
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
for k = 1:4
    plot(x, Cq_cum_cases(k,:), 'LineStyle', line_styles{k}, 'Color', line_colors{k}, 'LineWidth', 1.6);
end
xlabel('r/R', 'Color', 'k'); ylabel('C_Q (cum)', 'Color', 'k'); title('Q1(d): Total torque coefficient', 'Color', 'k');
legend(case_names, 'Location','best', 'TextColor', 'k', 'Box', 'off');


%% ========================================================================
%% Q2 -- Prandtl tip-loss 
%% ========================================================================
tol_lambda = 1e-8;       
max_iter_F = 200;        

root_fun_no_tip = @(th) inner_bemt_theta75(th,'no_tip', x, r, R, c_root, Nb, rho, Omega, C_l_alpha, Cdo, tol_lambda, max_iter_F) - T_target;
guess = 8; a = 0; b = 40; if root_fun_no_tip(a) * root_fun_no_tip(b) > 0, a = -10; b = 60; end
theta75_no_tip = fzero(root_fun_no_tip, guess);
[T_no_tip, lambda_no_tip, dTdr_no_tip, dQdr_total_no_tip, ~, ~] = inner_bemt_theta75(theta75_no_tip,'no_tip', x, r, R, c_root, Nb, rho, Omega, C_l_alpha, Cdo, tol_lambda, max_iter_F);

root_fun_with_tip = @(th) inner_bemt_theta75(th,'with_tip', x, r, R, c_root, Nb, rho, Omega, C_l_alpha, Cdo, tol_lambda, max_iter_F) - T_target;
if root_fun_with_tip(a) * root_fun_with_tip(b) > 0, a = -10; b = 60; end
theta75_with_tip = fzero(root_fun_with_tip, guess);
[T_tip, lambda_tip, dTdr_tip, dQdr_total_tip, ~, ~] = inner_bemt_theta75(theta75_with_tip,'with_tip', x, r, R, c_root, Nb, rho, Omega, C_l_alpha, Cdo, tol_lambda, max_iter_F);

dCTdr_no_tip = dTdr_no_tip ./ denom_CT;       
dCTdr_tip    = dTdr_tip    ./ denom_CT;
Cq_cum_no_tip = cumtrapz(r, dQdr_total_no_tip ./ denom_CQ);
Cq_cum_tip    = cumtrapz(r, dQdr_total_tip ./ denom_CQ);

% Save to CSV Data
ResultsData(end+1,:) = {'Q2', 'Theta75 (deg)', 'No Tip Loss', theta75_no_tip};
ResultsData(end+1,:) = {'Q2', 'Theta75 (deg)', 'With Tip Loss', theta75_with_tip};
ResultsData(end+1,:) = {'Q2', 'Achieved Thrust (N)', 'No Tip Loss', T_no_tip};
ResultsData(end+1,:) = {'Q2', 'Achieved Thrust (N)', 'With Tip Loss', T_tip};
ResultsData(end+1,:) = {'Q2', 'Lambda at 0.75', 'No Tip Loss', lambda_no_tip(idx75)};
ResultsData(end+1,:) = {'Q2', 'Lambda at 0.75', 'With Tip Loss', lambda_tip(idx75)};
ResultsData(end+1,:) = {'Q2', 'dCTdr at 0.75', 'No Tip Loss', dCTdr_no_tip(idx75)};
ResultsData(end+1,:) = {'Q2', 'dCTdr at 0.75', 'With Tip Loss', dCTdr_tip(idx75)};
ResultsData(end+1,:) = {'Q2', 'Total C_Q at R', 'No Tip Loss', Cq_cum_no_tip(end)};
ResultsData(end+1,:) = {'Q2', 'Total C_Q at R', 'With Tip Loss', Cq_cum_tip(end)};

% Plot 7
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x, lambda_no_tip, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 1.6);
plot(x, lambda_tip, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 1.6);
xlabel('r/R', 'Color', 'k'); ylabel('\lambda', 'Color', 'k'); title('Q2: Induced Inflow', 'Color', 'k');
legend({'No tip loss', 'With tip loss'}, 'Location','best', 'TextColor', 'k', 'Box', 'off');

% Plot 8
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x, dCTdr_no_tip, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 1.6);
plot(x, dCTdr_tip, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 1.6);
xlabel('r/R', 'Color', 'k'); ylabel('dC_T/dr', 'Color', 'k'); title('Q2: Thrust coefficient density', 'Color', 'k');
legend({'No tip loss', 'With tip loss'}, 'Location','best', 'TextColor', 'k', 'Box', 'off');

% Plot 9
nexttile; hold on; grid on; 
set(gca,'FontSize',10, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8]);
plot(x, Cq_cum_no_tip, 'Color', line_colors{1}, 'LineStyle', '-', 'LineWidth', 1.6);
plot(x, Cq_cum_tip, 'Color', line_colors{2}, 'LineStyle', '--', 'LineWidth', 1.6);
xlabel('r/R', 'Color', 'k'); ylabel('C_Q (cum)', 'Color', 'k'); title('Q2: Total Torque (cum)', 'Color', 'k');
legend({'No tip loss', 'With tip loss'}, 'Location','best', 'TextColor', 'k', 'Box', 'off');

%% ========================================================================
%% FINAL EXPORTS: Save SVG and CSV
%% ========================================================================

% 1. Save all plots into a single SVG file
exportgraphics(master_fig, 'All_Figures.svg', 'BackgroundColor', 'white', 'ContentType', 'vector');
disp('Successfully saved all figures to: All_Figures.svg');

% 2. Save all output metrics into a single CSV file
ResultsTable = cell2table(ResultsData, 'VariableNames', {'Question', 'Metric', 'Case', 'Value'});
writetable(ResultsTable, 'BEMT_Results_Summary.csv');
disp('Successfully saved all calculated outputs to: BEMT_Results_Summary.csv');


%% ========================================================================
%% LOCAL FUNCTIONS (Must remain at the very end of the file)
%% ========================================================================

function T = total_thrust_for_theta75(theta75_deg, case_id, x, r, R, c_root, Nb, rho, Omega, C_l_alpha)
    theta75_rad = deg2rad(theta75_deg);
    switch case_id
        case 1, theta_x = theta75_rad .* (0.75 ./ x); c_x = c_root * ones(size(x));
        case 2, theta_x = theta75_rad * ones(size(x)); c_x = c_root * ones(size(x));
        case 3
            m = (deg2rad(theta75_deg - 15) - theta75_rad) / (1 - 0.75);
            theta_x = m .* x + (theta75_rad - m * 0.75); c_x = c_root * ones(size(x));
        case 4
            theta_x = theta75_rad .* (0.75 ./ x);
            c_x = c_root + (0.5 * c_root - c_root) .* ((x - x(1)) ./ (1 - x(1)));
    end
    sigma_x = Nb .* c_x ./ (pi * R);
    f_x = zeros(size(x));
    for i = 1:numel(x)
        A = 4 * x(i); B = 0.5 * sigma_x(i) * C_l_alpha * x(i); C = 0.5 * sigma_x(i) * C_l_alpha * x(i)^2 * theta_x(i);
        disc = max(B^2 + 4 * A * C, 0);
        lambda_i = (-B + sqrt(disc)) / (2 * A);
        f_x(i) = 0.5 * rho * Nb * (Omega * r(i))^2 * (C_l_alpha * (theta_x(i) - lambda_i / x(i))) * c_x(i);
    end
    T = trapz(r, f_x);
end

function [T, lambda_x, dTdr, dQdr_total, dQdr_ind, dQdr_prof] = inner_bemt_theta75(theta75_deg, mode, x, r, R, c_root, Nb, rho, Omega, C_l_alpha, Cdo, tol_lambda, max_iter_F)
    theta_x = deg2rad(theta75_deg) .* ones(size(x)); c_x = c_root .* ones(size(x)); sigma_x = Nb .* c_x ./ (pi * R);
    lambda_from_F_theta = @(sigma, th, F, x_l, Cla) ( sigma .* Cla ./ 16 .* ( sqrt(1 + (32 .* F .* th .* x_l) ./ (sigma .* Cla) ) - 1 ) );
    
    F = ones(size(x)); lambda_old = lambda_from_F_theta(sigma_x, theta_x, F, x, C_l_alpha);
    if strcmp(mode,'with_tip')
        for it = 1:max_iter_F
            sinphi = max(sin(atan(lambda_old ./ x)), 1e-12);
            F_new = max(min((2/pi) .* acos( exp(-(Nb/2) .* ((1 - x) ./ (x .* sinphi))) ), 1), 0);
            lambda_new = lambda_from_F_theta(sigma_x, theta_x, F_new, x, C_l_alpha);
            if max(abs(lambda_new - lambda_old)) < tol_lambda, lambda_old = lambda_new; break; end
            lambda_old = lambda_new;
        end
    end
    lambda_x = lambda_old;
    dTdr = 0.5 .* rho .* Nb .* (Omega .* r).^2 .* (C_l_alpha .* (theta_x - lambda_x ./ x)) .* c_x;   
    dQdr_prof = r .* (0.5 .* rho .* Nb .* (Omega .* r).^2 .* c_x .* Cdo); 
    dQdr_ind = dTdr .* lambda_x .* R;
    dQdr_total = dQdr_prof + dQdr_ind;
    T = trapz(r, dTdr);
end