function dx = tii_plant_dyn(t, x, u, sc, P, tau, drate)
% TII_PLANT_DYN  Dynamic single-track (bicycle) AGV in the Frenet frame, the
% plant of the gate-5 transfer check. State x = [e_y; e_psi; delta; s; v_y; r]
% with body lateral velocity v_y and yaw rate r. Linear tyres with lateral
% force saturation at mu*F_z; longitudinal speed v_x(t) is imposed (as in the
% kinematic plant). The controllers are NOT changed and still assume the
% kinematic model; the dynamic effects (tyre slip, yaw inertia, understeer)
% act on them as unmodelled dynamics. Polarity and gain mismatch act on the
% wheel angle, delta_w = b * g_s * delta. Validated open loop against the
% Vehicle Dynamics Blockset 3DOF single-track block (tii_vdbs_check).
V = P.V; if isfield(sc, 'V'), f = fieldnames(sc.V); for i = 1:numel(f), V.(f{i}) = sc.V.(f{i}); end, end
ey = x(1); ep = x(2); de = x(3); s = x(4); vy = x(5); r = x(6);
vx = max(tii_speed(sc, t), 0.05);
kap = tii_kappa(sc, s);
b = sc.b0; if t >= sc.tflip, b = -b; end
dy = sc.dy * sin(sc.dw * t + sc.dph);
dp = sc.dpsi * sin(1.3 * sc.dw * t + 0.5 * sc.dph);
L = V.a + V.b;
Fzf = V.m * V.g * V.b / L; Fzr = V.m * V.g * V.a / L;
Cf = V.cbar * V.cfr * Fzf; Cr = V.cbar * Fzr;
dw = b * sc.gscale * de;                                   % applied wheel angle
af = dw - atan((vy + V.a * r) / vx);                       % slip angles
ar = -atan((vy - V.b * r) / vx);
Fyf = min(max(Cf * af, -V.mu * Fzf), V.mu * Fzf);          % lateral force saturation
Fyr = min(max(Cr * ar, -V.mu * Fzr), V.mu * Fzr);
den = max(1 - kap * ey, 0.2);
sdot = (vx * cos(ep) - vy * sin(ep)) / den;
dx = zeros(6, 1);
dx(1) = vx * sin(ep) + vy * cos(ep) + dy;
dx(2) = r - kap * sdot + dp;
dx(3) = min(max((u - de) / tau, -drate), drate);
dx(4) = sdot;
dx(5) = (Fyf * cos(dw) + Fyr) / V.m - vx * r;
dx(6) = (V.a * Fyf * cos(dw) - V.b * Fyr) / V.Izz;
end
