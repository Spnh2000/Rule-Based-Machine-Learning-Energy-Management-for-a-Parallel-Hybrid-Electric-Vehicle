function stateCE = state_CE(input)
% Engine state determination (0 = off, 1 = on)

T_MGB = input(1);    % Gearbox torque
u     = input(2);    % Torque-split ratio

% Default
stateCE = 1;

switch true
    case (T_MGB <= 0)
        % Regeneration or stop → engine off
        stateCE = 0;
        
    case (T_MGB > 0 && u == 1)
        % Electric drive → engine off
        stateCE = 0;
        
    otherwise
        % Load point shifting / engine drive → engine on
        stateCE = 1;
end
