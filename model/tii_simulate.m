function R = tii_simulate(sc, ctrl, P, keeplog)
% TII_SIMULATE  Closed-loop simulation of one controller on one scenario.
%   Plant: Frenet AGV error dynamics with curvature, speed profile, unknown
%   steering polarity b(t), first-order steering lag, rate limit,
%   saturation, matched/unmatched disturbances, measurement noise and bias.
%   Plant is integrated with RK4 at dtc/nsub; controller runs at dtc with
%   zero-order hold. Safety is evaluated on the TRUE state.
if nargin < 4, keeplog = false; end

dtc = P.dtc; nsub = P.nsub; dtp = dtc / nsub;
N = round(P.T / dtc);
tau = P.tau * sc.tau_scale; drate = P.drate * sc.rate_scale; dmax = P.dmax;

noise_offset = 7919;   % measurement-noise seed = scenario seed + 7919 (as in the paper's runs)
rng(sc.seed + noise_offset, 'twister');
nz = randn(2, N);

x = sc.x0;
% plant: kinematic single-track (default) or dynamic single-track (gate 5)
rhs = @plant_rhs;
if isfield(sc, 'plant') && strcmp(sc.plant, 'dyn')
    rhs = @tii_plant_dyn; x = [x(:); 0; 0];             % + body lateral velocity, yaw rate
end
nd = 0; if isfield(sc, 'udelay'), nd = sc.udelay; end    % command delay in control periods
ubuf = zeros(1, nd + 1);
mem = [];
gam = zeros(1, N); uc = zeros(1, N); ey = zeros(1, N); ep = zeros(1, N);
cpu = zeros(1, N); tt = (0:N-1) * dtc;
if keeplog
    LG.F1 = zeros(1, N); LG.co = zeros(1, N); LG.delta = zeros(1, N);
    LG.s = nan(1, N); LG.N = nan(1, N); LG.th = nan(2, N); LG.a1 = nan(1, N);
    LG.F2 = nan(1, N); LG.kap = nan(1, N); LG.rho = nan(1, N); LG.V = nan(1, N); LG.phi = nan(1, N);
    LG.sig = nan(1, N);  % Polarity signal for monitor diagnosis
    LG.trig = zeros(1, N);  % switch trigger: 1 evidence-gated, 2 hard backup
    LG.Q = nan(1, N); LG.y = nan(1, N); LG.r = nan(1, N);   % monitor evidence (filtered excitation, window pair)
    LG.bt = nan(1, N);      % true polarity b(t)
end
info_acc = struct('aclip', 0, 'q2clip', 0, 'smax', 0, 'tsw', 0, 'Nsign0', NaN, 'kumin', 1, 'nsw', 0, 'sclip', 0);
ok = true; lastN = NaN;

for k = 1:N
    t = tt(k);
    [F1, co] = tii_corridor(sc, t);
    % --- true-state safety -----------------------------------------------
    gam(k) = abs(x(1) - co) / F1;
    ey(k) = x(1) - co; ep(k) = x(2);
    if ~all(isfinite(x)) || gam(k) > 5
        ok = false; gam(k:end) = max(gam(k), 5); break;
    end
    % --- measurements -----------------------------------------------------
    m.t = t; m.F1 = F1; m.co = co; m.v = tii_speed(sc, t) * sc.vscale;
    m.ey = x(1) + sc.by + sc.ny * nz(1, k);
    m.epsi = x(2) + sc.bpsi + sc.npsi * nz(2, k);
    m.kappa = tii_kappa(sc, x(4));       % map curvature (used by MPC/Stanley)
    m.delta = x(3) + sc.dbias;           % steering encoder (offset, quantisation: stage 'sensor')
    if sc.dquant > 0, m.delta = sc.dquant * round(m.delta / sc.dquant); end
    m.dtc = dtc;
    % --- true polarity (for oracle only) ----------------------------------
    m.b_true = sc.b0; if t >= sc.tflip, m.b_true = -m.b_true; end
    % --- controller --------------------------------------------------------
    tic;
    [ucmd, mem, inf_k] = ctrl.fn(m, mem, P, ctrl);
    cpu(k) = toc;
    if ~isfinite(ucmd), ucmd = 0; end
    ucmd = min(max(ucmd, -dmax), dmax);
    uc(k) = ucmd;
    % --- controller diagnostics ------------------------------------------
    if isfield(inf_k, 'aclip'), info_acc.aclip = info_acc.aclip + inf_k.aclip; end
    if isfield(inf_k, 'q2clip'), info_acc.q2clip = info_acc.q2clip + inf_k.q2clip; end
    if isfield(inf_k, 'sclip'), info_acc.sclip = info_acc.sclip + inf_k.sclip; end
    if isfield(inf_k, 's')
        info_acc.smax = max(info_acc.smax, abs(inf_k.s));
        sN = sign(inf_k.N);
        if isnan(info_acc.Nsign0), info_acc.Nsign0 = sN; end
        if ~isnan(lastN) && sN ~= lastN, info_acc.tsw = t; info_acc.nsw = info_acc.nsw + 1; end
        lastN = sN;
    end
    if keeplog
        LG.F1(k) = F1; LG.co(k) = co; LG.delta(k) = x(3);
        if isfield(inf_k, 's'), LG.s(k) = inf_k.s; LG.N(k) = inf_k.N; end
        if isfield(inf_k, 'th'), LG.th(:, k) = inf_k.th(:); end
        if isfield(inf_k, 'a1'), LG.a1(k) = inf_k.a1; end
        if isfield(inf_k, 'F2'), LG.F2(k) = inf_k.F2; end
        if isfield(inf_k, 'rho'), LG.rho(k) = inf_k.rho; LG.V(k) = inf_k.V; LG.phi(k) = inf_k.phi; end
        if isfield(inf_k, 'sig'), LG.sig(k) = inf_k.sig; end
        if isfield(inf_k, 'trig'), LG.trig(k) = inf_k.trig; end
        if isfield(inf_k, 'Q'), LG.Q(k) = inf_k.Q; LG.y(k) = inf_k.y; LG.r(k) = inf_k.r; end
        LG.bt(k) = m.b_true;
    end
    % --- plant (RK4, ZOH input) -------------------------------------------
    ubuf = [ucmd, ubuf(1:end-1)]; uap = ubuf(end);      % applied command (delayed by nd samples)
    for j = 1:nsub
        tp = t + (j - 1) * dtp;
        k1 = rhs(tp, x, uap, sc, P, tau, drate);
        k2 = rhs(tp + dtp/2, x + dtp/2*k1, uap, sc, P, tau, drate);
        k3 = rhs(tp + dtp/2, x + dtp/2*k2, uap, sc, P, tau, drate);
        k4 = rhs(tp + dtp, x + dtp*k3, uap, sc, P, tau, drate);
        x = x + dtp/6 * (k1 + 2*k2 + 2*k3 + k4);
        x(3) = min(max(x(3), -dmax), dmax);
    end
end

% --- metrics ------------------------------------------------------------
it = tt >= P.T - P.tail;
R.name = ctrl.name; R.scen = sc.name; R.b = sc.b0; R.seed = sc.seed;
R.ok = ok;
R.gmax = max(gam);
R.success = ok && R.gmax < 1;
R.viol_time = sum(gam >= 1) * dtc;
R.rmse_tail = sqrt(mean(ey(it).^2));
R.rmse_tail_psi = sqrt(mean(ep(it).^2));        % heading error over the same tail window
R.urate = sqrt(mean((diff(uc) / dtc).^2));      % RMS rate of the steering command (rad/s)
R.rmse = sqrt(mean(ey.^2));
R.urms = sqrt(mean(uc.^2));
R.usat = mean(abs(uc) >= dmax - 1e-6);
R.TV = sum(abs(diff(uc)));
R.cpu_mean = mean(cpu(cpu > 0)); R.cpu_max = max(cpu);
R.smax = info_acc.smax; R.tsw = info_acc.tsw; R.nsw = info_acc.nsw; R.sclip = info_acc.sclip / N;
R.aclip = info_acc.aclip / N; R.q2clip = info_acc.q2clip / N;
if keeplog
    LG.t = tt; LG.gam = gam; LG.ey = ey; LG.ep = ep; LG.u = uc;
    R.log = LG;
end
end

function dx = plant_rhs(t, x, u, sc, P, tau, drate)
ey = x(1); ep = x(2); de = x(3); s = x(4);
v = tii_speed(sc, t);
kap = tii_kappa(sc, s);
b = sc.b0; if t >= sc.tflip, b = -b; end
dy = sc.dy * sin(sc.dw * t + sc.dph);
dp = sc.dpsi * sin(1.3 * sc.dw * t + 0.5 * sc.dph);
den = max(1 - kap * ey, 0.2);
dx = zeros(4, 1);
dx(1) = v * sin(ep) + dy;
dx(2) = b * sc.gscale * v / P.L * tan(de) - v * kap * cos(ep) / den + dp;
dx(3) = min(max((u - de) / tau, -drate), drate);
dx(4) = v * cos(ep) / den;
end
