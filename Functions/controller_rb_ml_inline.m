function u = controller_rb_ml_inline(input)

%% Inputs
w_MGB = input(1);
dw_MGB = input(2);
T_MGB = input(3);
Q_BT  = input(6);

%% Variables / Globals (provided by your init scripts)
global w_EM_max T_EM_max Q_BT_IC SOC_th
theta_EM  = 0.1;
epsilon   = 0.01;
u_LPS_max = 0.3;

% ---- Select cycle thresholds ----
current_system = get_param(0,'CurrentSystem');
stop_time = str2double(get_param(current_system,'StopTime'));
if stop_time == 1220            % NEDC
    T_MGB_th = 60; 
    T_ED = 29;
    u_LPS_min = -0.565;
    SOC_th = 0.30;

elseif stop_time == 1877         % FTP-75
    T_MGB_th = 110;
    T_ED = 33;
    u_LPS_min = -0.223;
    SOC_th = 0.30;

else
    % fallback (NEDC-like)
    T_MGB_th = 60;
    T_ED = 29;
    u_LPS_min = -0.565;
    SOC_th = 0.30;
end

%% -------------------------
%  Base RULE-BASED controller
% --------------------------

% Initialize control output
u = 0;

% Rule 1: LPS in motor mode
% Condition: High torque demand AND sufficient battery charge
if (T_MGB >= T_MGB_th) && (Q_BT > 0.8*Q_BT_IC)
    % Calculate maximum motor torque at current speed
    T_EM_available = interp1(w_EM_max, T_EM_max, w_MGB, 'linear', 'extrap');
    % Apply speed-dependent damping and safety margin
    T_EM_effective = T_EM_available - abs(theta_EM * dw_MGB) - epsilon;
    % Set control output with upper bound constraint
    u = min(T_EM_effective / max(T_MGB, eps), u_LPS_max);

% Rule 2: LPS in generator mode  
% Condition: Medium torque demand (between engine drag and threshold)
elseif (T_MGB > T_ED) && (T_MGB < T_MGB_th)
    % Calculate maximum generator torque at current speed
    T_EG_available = interp1(w_EM_max, -T_EM_max, w_MGB, 'linear', 'extrap');
    % Apply speed-dependent damping and safety margin
    T_EG_effective = T_EG_available + abs(theta_EM * dw_MGB) + epsilon;
    % Set control output with lower bound constraint
    u = max(T_EG_effective / max(T_MGB, eps), u_LPS_min);

% Rule 3: Regeneration mode
% Condition: Negative torque (braking/deceleration)
elseif (T_MGB < 0)
    % Calculate maximum generator torque for regeneration
    T_EG_regen = interp1(w_EM_max, -T_EM_max, w_MGB, 'linear', 'extrap');
    % Apply speed-dependent damping and safety margin
    T_EG_regen_effective = T_EG_regen + abs(theta_EM * dw_MGB) + epsilon;
    % Set control output with upper bound of 1 (full regeneration)
    u = min(T_EG_regen_effective / min(T_MGB, -eps), 1);

% Rule 4: Pure Electric Drive mode
% Condition: Low torque demand AND sufficient battery state of charge
elseif (T_MGB > 0) && (T_MGB < T_ED) && (Q_BT >= SOC_th * Q_BT_IC)
    % Full electric drive - no engine assistance needed
    u = 1;

% Rule 5: Engine mode (default)
% Condition: All other cases
else
    % Conventional engine operation - no electric assistance
    u = 0;
end

%% -------------------------
%  Tiny ML augmentation (optional, simple)
%  - Only nudge when pulling torque (T_MGB>0) and NOT pure ED (u<1)
% --------------------------
if (T_MGB > 0) && (u < 1) && (Q_BT > 0)
    % Persistent 3-weight linear model
    persistent w_ml soc_ref eta alpha deadband
    if isempty(w_ml)
        w_ml = zeros(3,1);   % [bias; k_SOC; k_load]
        soc_ref = 0.50;      % aim ≈ 50% relative charge
        eta  = 5e-3;         % learning rate
        alpha = 0.3;         % blend factor
        deadband = 0.01;     % 1% SOC deadband
    end

    % Relative "SOC" proxy (use initial charge Q_BT_IC)
    soc_rel = max(min(Q_BT / max(Q_BT_IC,eps),1),0);
    e_soc   = soc_ref - soc_rel;         % >0 ⇒ SOC low (use less battery)
    load_n  = min(max( (T_MGB - T_ED) / max(T_MGB_th - T_ED,1e-6), 0), 1);

    phi = [1; e_soc; load_n];
    Delta_u_ml = w_ml.' * phi;

    % --- Clamp to EM torque limits + your LPS bounds ---
    Tmax = interp1(w_EM_max, T_EM_max, w_MGB, 'linear','extrap');
    TEM_pos_max =  Tmax - abs(theta_EM*dw_MGB) - epsilon;
    TEM_neg_min = -Tmax + abs(theta_EM*dw_MGB) + epsilon;

    if T_MGB ~= 0
        u_hi = min(u_LPS_max, TEM_pos_max / T_MGB);
        u_lo = max(u_LPS_min, TEM_neg_min / T_MGB);
    else
        u_hi = u_LPS_max; u_lo = u_LPS_min;
    end

    % Blend and clamp
    u = min(max(u + alpha*Delta_u_ml, u_lo), u_hi);

    % Simple SOC-driven LMS update (no ECMS)
    if abs(e_soc) > deadband
        w_ml = 0.999*w_ml - eta * e_soc * phi;
    end
end
end
