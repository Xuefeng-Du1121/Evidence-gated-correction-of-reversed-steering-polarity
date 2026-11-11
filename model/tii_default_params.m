function P = tii_default_params()
% TII_DEFAULT_PARAMS  Plant, simulation, and BCN controller parameters used in
% the paper. The monitor values mbeta, mlam, meps, mrho are the setting
% selected by the common tuning protocol (Appendix C of the paper); all
% reported BCN results use exactly these values.

%% Simulation
P.dtc  = 0.02;          % controller sampling time (50 Hz)
P.nsub = 10;            % plant RK4 substeps per control step (dt_p = 2 ms)
P.T    = 30;            % horizon (s)
P.tail = 10;            % window for tail RMSE (s)

%% Vehicle / actuator (nominal values known to the controllers)
P.L     = 0.8;              % wheelbase (m)
P.dmax  = deg2rad(30);      % steering saturation (rad)
P.drate = deg2rad(90);      % steering rate limit (rad/s)
P.tau   = 0.10;             % steering actuator lag (s)

%% BCN: barrier-coordinate tracking layers and the polarity monitor
C.k1    = 1.0;     % layer-1 gain
C.kap1  = 0.50;    % layer-1 cross-coupling damping (Young domination)
C.gam1  = 0.02;    % layer-1 adaptation rate
C.k2    = 4.0;     % layer-2 gain (must exceed ell)
C.gam2  = 0.05;    % layer-2 adaptation rate
C.sig   = 0.05;    % sigma-modification leakage
C.eta1  = 1.00;    % Young constant of the layer-1 robust term
C.eta2  = 1.00;    % Young constant of the layer-2 robust term
C.abar  = 0.40;    % virtual heading-target limit (feasibility condition)
C.rho2  = 0.70;    % lower bound of the dynamic heading boundary F2 (rad)
C.r2    = 0.60;    % feasibility ratio |alpha1|/F2 < r2
C.epsF  = 0.01;    % smoothing of |.| in F2
C.delF  = 0.02;    % smoothing of [.]_+ in F2
C.varpi = 1.0;     % Nussbaum search rate (ablation BCN-Nuss only)
C.eN    = 0.5;     % Nussbaum amplitude (ablation BCN-Nuss only)
C.cN    = 1.0;     % Nussbaum growth: N = eN*exp(cN s)cos(s) (ablation BCN-Nuss only)
C.qmax  = 0.999;   % numerical guard for atanh arguments
C.thmax = [2; 2];  % projection bounds of theta-hat
C.dsmax = 0.2;     % max Nussbaum increment per sample (ablation BCN-Nuss only)
C.dlt   = 0.05;    % dead zone on chi2 for the Nussbaum search (ablation BCN-Nuss only)
C.Nbar  = 1.0;     % monitor: bounded gain magnitude |N|
C.mbeta = 1.1486570024932701;     % monitor: envelope factor (tuned)
C.mlam  = 0.82217217320189817;    % monitor: envelope decay rate (1/s) (tuned)
C.meps  = 0.0059599243596125933;  % monitor: envelope offset increment (tuned)
C.mdwell = 0.5;    % monitor: dwell time after a switch (s)
C.mw1   = 1.0;     % monitor: weight of chi1 in the monitored V
C.mreset = true;   % monitor: reset adaptive estimates at a switch
C.mbeta2 = 20;     % monitor: hard (evidence-free) envelope factor
C.mtf   = 0.2;     % monitor: evidence filter time constant (s)
C.mQ0   = 1e-5;    % monitor: minimum excitation for evidence
C.mwin  = 0.16;    % monitor: evidence window W (s) = 8 control periods
C.mrho  = 0.35062102331122685;    % monitor: evidence threshold (tuned)
P.C = C;

%% Dynamic single-track vehicle (plant-transfer check; not known to the
%% controller). Used only when scenario field sc.plant = 'dyn'.
V.m     = 300;      % mass (kg), AGV class
V.Izz   = 35;       % yaw inertia (kg m^2)
V.a     = 0.40;     % CG to front axle (m); V.a + V.b = L
V.b     = 0.40;     % CG to rear axle (m)
V.cbar  = 8;        % normalised rear cornering stiffness C_r / F_zr (1/rad)
V.cfr   = 0.8;      % front/rear normalised stiffness ratio (<1: understeer)
V.mu    = 0.6;      % floor friction coefficient (lateral force saturation)
V.g     = 9.81;
P.V = V;
end
