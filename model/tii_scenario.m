function sc = tii_scenario(name, b, seed)
% TII_SCENARIO  Build one Monte-Carlo realisation of a benchmark scenario.
%   sc = tii_scenario(name, b, seed)
%   name : 'S1'..'S7' (see list below), b : true steering polarity (+1/-1)
%   seed : Monte-Carlo seed (randomises initial state, noise, event timing)
%
%   S1 Nominal         curved path, nominal lane, mild noise
%   S2 NearBoundary    starts at 75-85% of the corridor, heading outward 2-5 deg
%   S3 PalletIntrusion sudden one-sided obstacle during the sign-search transient
%   S4 MovingForklift  oscillating obstacle offset c_o(t) and width F1(t)
%   S5 CurvyFast       high curvature, variable speed 0.8-1.6 m/s
%   S6 Harsh           S3 + bias, noise, disturbances, gain mismatch, slow actuator
%   S7 PolarityFlip    b flips sign at t=15 s (outside assumptions, stress test)

% Default arguments
if nargin < 1, name = 'S1'; end
if nargin < 2, b = +1; end
if nargin < 3, seed = 1; end

rng(seed, 'twister');
u = @(a, c) a + (c - a) * rand();

sc.name = name; sc.b0 = b; sc.seed = seed;

% ---- defaults --------------------------------------------------------
sc.v0 = 1.2; sc.va = 0; sc.vw = 0;          % v = v0 + va*sin(vw t)
sc.ka = 0.20; sc.kw = 2*pi/25; sc.kph = u(0, 2*pi);   % kappa(s)
sc.W0 = 0.45;  sc.Wa = 0; sc.Ww = 0; sc.Wph = 0;      % nominal half width
sc.ev = [];                                  % obstacle events
sc.fork = [];                                % moving obstacle
sc.ny = 0.01; sc.npsi = deg2rad(0.5);        % measurement noise std
sc.by = 0;    sc.bpsi = 0;                   % measurement bias
sc.dy = 0;    sc.dpsi = 0; sc.dw = 0.7; sc.dph = u(0, 2*pi);  % disturbances
sc.gscale = 1;                               % true/nominal steering gain
sc.tau_scale = 1; sc.rate_scale = 1;         % actuator mismatch
sc.tflip = inf;                              % polarity flip time
sc.dbias = 0; sc.dquant = 0; sc.vscale = 1;  % encoder offset (rad), encoder step (rad), measured/true speed
                                             % (stage 'sensor'; set without drawing from the RNG)

% initial state (fraction of corridor, heading)
q0 = u(-0.4, 0.4); psi0 = deg2rad(u(-10, 10));

switch name
    case 'S1'
    case 'S2'
        sg = sign(u(-1, 1)); if sg == 0, sg = 1; end
        q0 = sg * u(0.75, 0.85); psi0 = sg * deg2rad(u(2, 5));
    case 'S3'
        q0 = u(0.2, 0.4);
        sc.ev = struct('t0', u(0.8, 1.6), 'dur', u(4, 6), 'tr', u(0.6, 1.0), ...
                       'h', 0.25, 'side', +1);
    case 'S4'
        sc.fork = struct('A', 0.08, 'w', 2*pi/u(5, 7), 'ph', u(0, 2*pi), 'Wa', 0.06);
        sc.ka = 0.15;
    case 'S5'
        sc.ka = 0.40; sc.kw = 2*pi/12; sc.v0 = 1.2; sc.va = 0.4; sc.vw = 2*pi/10;
        sc.W0 = 0.40;
    case 'S6'
        q0 = u(0.2, 0.4);
        sc.ev = struct('t0', u(0.8, 1.6), 'dur', u(4, 6), 'tr', u(0.6, 1.0), ...
                       'h', 0.25, 'side', +1);
        sc.ny = 0.02; sc.npsi = deg2rad(1.0);
        sc.by = u(-0.02, 0.02); sc.bpsi = deg2rad(u(-1, 1));
        sc.dy = 0.03; sc.dpsi = 0.05;
        sc.gscale = u(0.6, 1.4); sc.tau_scale = 2; sc.rate_scale = 0.75;
    case 'S7'
        sc.tflip = 15;
    case 'S8'                                % fast slalom intrusions (both sides)
        q0 = u(-0.3, 0.3);
        t1 = u(2, 3); t2 = t1 + u(5, 6);
        sc.ev = [struct('t0', t1, 'dur', u(2.5, 3.5), 'tr', u(0.3, 0.5), 'h', 0.35, 'side', +1), ...
                 struct('t0', t2, 'dur', u(2.5, 3.5), 'tr', u(0.3, 0.5), 'h', 0.35, 'side', -1)];
        sc.ka = 0.25;
    otherwise
        error('unknown scenario %s', name);
end

[F10, co0] = tii_corridor(sc, 0);
sc.x0 = [co0 + q0 * F10; psi0; 0; 0];   % [e_y; e_psi; delta; s]
end
