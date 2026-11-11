function [u, mem, info] = ctrl_bcn(m, mem, P, ctrl)
% CTRL_BCN  Barrier-coordinate Nussbaum (BCN) controller, exactly as in the
% manuscript (Section IV).  Options in ctrl:
%   .barrier (true)  : use barrier coordinates chi_i = F_i*atanh(x_i/F_i)
%   .dynF2   (true)  : dynamic heading boundary F2(alpha1); else F2 = rho2
%   .nuss    'search' : Nussbaum gain N(s)
%            'monitor': envelope-gated polarity monitor (proposed)
%            'nominal': N = -ctrl.bhat, never adapted (hand-calibrated sign)
%            'oracle' : N = -b_true(t), true sign at every instant
%
% Layer 1 (known sign, no Nussbaum):
%   w1 = a1*chi1,  nu1 = (k1 + kap1 + th1/(2 eta1^2)) w1/(v*g_lo)
%   alpha1 = -abar*tanh(nu1/abar)
% Layer 2 (unknown sign):
%   nu2 = (k2 + th2 Phi2^2/(2 eta2^2)) chi2
%   u = N(s) nu2/(a2*v/L),  ds = varpi*chi2*nu2 >= 0
% Adaptation: dth1 = gam1 w1^2/(2eta1^2) - sig th1,
%             dth2 = gam2 Phi2^2 chi2^2/(2eta2^2) - sig th2

C = P.C; dt = m.dtc;
if ctrl.noenv                             % BCN-noEnv: its own evidence threshold, dwell, floor
    C.mrho = P.NE.mrho; C.mdwell = P.NE.mdwell; C.mQ0 = P.NE.mQ0;
end
if isempty(mem), mem.s = 0; mem.th = [0; 0]; end
v = max(m.v, 0.05);
qm = C.qmax;

if ctrl.dynF2, F2max = max(C.rho2, C.abar / C.r2) + C.delF; else, F2max = C.rho2; end
glo = sin(F2max) / F2max;                 % lower bound of sinc(x2)
c1 = 1 / (v * glo);

%% ---------------- layer 1 -------------------------------------------------
F1 = m.F1; x1 = m.ey - m.co; x2 = m.epsi;
if ctrl.barrier
    q1 = min(max(x1 / F1, -qm), qm);
    at1 = atanh(q1); a1 = 1 / (1 - q1^2);
    chi1 = F1 * at1; w1 = a1 * chi1;
    dw_dx = a1^2 * (1 + 2 * q1 * at1);              % dw1/dx1
    dw_dF = a1 * at1 - q1 * a1^2 * (1 + 2 * q1 * at1);  % dw1/dF1
else
    chi1 = x1; w1 = x1; dw_dx = 1; dw_dF = 0;
end
th1 = mem.th(1); th2 = mem.th(2);
e1 = 1 / (2 * C.eta1^2);
nu1 = c1 * (C.k1 + C.kap1 + th1 * e1) * w1;
alpha1 = -C.abar * tanh(nu1 / C.abar);
H1 = e1 * w1^2;
dth1 = C.gam1 * H1 - C.sig * th1;

% d(alpha1)/d(nu1) and partials of nu1
sech2 = 1 - tanh(nu1 / C.abar)^2;
dal_dw = -sech2 * c1 * (C.k1 + C.kap1 + th1 * e1);
dal_dth = -sech2 * c1 * e1 * w1;
% known part of alpha1-dot (x1-dot nominal part v*sin(x2), theta1-hat dynamics)
K = dal_dw * dw_dx * v * sin(x2) + dal_dth * dth1;
% magnitude multiplying the unknown rates (d_y, c_o-dot, F1-dot, v-dot)
D = abs(dal_dw) * (abs(dw_dx) + abs(dw_dF)) + abs(alpha1) / v;

%% ---------------- layer 2 -------------------------------------------------
if ctrl.dynF2
    z = sqrt(alpha1^2 + C.epsF^2) / C.r2 - C.rho2;
    sp = 0.5 * (z + sqrt(z^2 + C.delF^2));           % smooth [z]_+
    F2 = C.rho2 + sp;
    dF2 = 0.5 * (1 + z / sqrt(z^2 + C.delF^2)) * alpha1 / (C.r2 * sqrt(alpha1^2 + C.epsF^2));
else
    F2 = C.rho2; dF2 = 0;
end
q2clip = 0;
if ctrl.barrier
    q2 = x2 / F2; qa = alpha1 / F2;
    if abs(q2) > qm, q2clip = 1; q2 = sign(q2) * qm; end
    qa = min(max(qa, -qm), qm);
    a2 = 1 / (1 - q2^2); aa = 1 / (1 - qa^2);
    B2 = atanh(q2) - q2 * a2; Ba = atanh(qa) - qa * aa;
    chi2 = F2 * (atanh(q2) - atanh(qa));
    Lam = aa + (abs(B2) + abs(Ba)) * abs(dF2);
else
    a2 = 1; chi2 = x2 - alpha1; Lam = 1;
end
Phi2 = a2 + Lam * (abs(K) + D);
e2 = 1 / (2 * C.eta2^2);
nu2 = (C.k2 + th2 * e2 * Phi2^2) * chi2;
H2 = e2 * Phi2^2 * chi2^2;
dth2 = C.gam2 * H2 - C.sig * th2;

switch ctrl.nuss
    case 'search'
        N = -ctrl.bhat * C.eN * exp(min(C.cN * mem.s, 700)) * cos(mem.s);   % type-B Nussbaum (Chen 2019), N(0) matches nominal sign
        % dead-zoned search: ds = varpi*(nu2/chi2)*[chi2^2 - dlt^2]_+ >= 0
        % (freezes s once |chi2| <= dlt so a persistent steering offset,
        %  e.g. curvature, cannot drive s out of a stabilising lobe)
        ds = C.varpi * (C.k2 + th2 * e2 * Phi2^2) * max(chi2^2 - C.dlt^2, 0);
    case 'monitor'
        % Barrier-envelope polarity monitor with bounded gain N = sig*Nbar
        % (sig = +1 <=> hypothesis b = -1).  A switch is triggered when
        %   (a) V > phi_k and the polarity evidence contradicts sig, or
        %   (b) V > mbeta2*phi_k (hard envelope, evidence-free backup),
        % with V = (mw1 chi1^2 + chi2^2)/2 and
        %   phi_k(t) = mbeta V(t_k) e^{-mlam (t-t_k)} + meps (k+1).
        % Evidence: filtered correlation of the drift-free heading increment
        % over a sliding window W with the integrated steering regressor,
        %   y = e_psi(t) - e_psi(t-W) - int drift,  r = int (v/L) tan(delta),
        %   rho = <y r> / <r^2>  ~  b*g     (noise/bias attenuated by W/dt).
        if ~isfield(mem, 'sig')
            mem.sig = -ctrl.bhat; mem.k = 0; mem.tk = m.t; mem.tsw = -inf;
            mem.Vk = 0.5 * (C.mw1 * chi1^2 + chi2^2);
            mem.pv = []; mem.Pq = 0; mem.Q = 0; mem.cd = 0; mem.cr = 0;
            mem.H = nan(3, max(round(C.mwin / dt), 1) + 1); mem.ih = 0;
        end
        if ~isempty(mem.pv)                  % accumulate drift and regressor
            mem.cd = mem.cd + mem.pv(2) * dt;
            mem.cr = mem.cr + mem.pv(3) * dt;
        end
        den = 1 - m.kappa * m.ey;
        mem.pv = [m.epsi, -v * m.kappa * cos(m.epsi) / den, v / P.L * tan(m.delta)];
        nh = size(mem.H, 2); mem.ih = mem.ih + 1;
        mem.H(:, mod(mem.ih - 1, nh) + 1) = [m.epsi; mem.cd; mem.cr];
        y = NaN; r = NaN;                    % logged for the evidence analysis (stage 'evid')
        if mem.ih > nh                       % window full: oldest sample
            ho = mem.H(:, mod(mem.ih, nh) + 1);
            y = (m.epsi - ho(1)) - (mem.cd - ho(2));
            r = mem.cr - ho(3);
            a = dt / C.mtf;
            mem.Pq = (1 - a) * mem.Pq + a * y * r;
            mem.Q  = (1 - a) * mem.Q  + a * r^2;
        end
        Q = mem.Q;
        rho = mem.Pq / max(Q, C.mQ0);                            % rho ~ b*g
        contra = (Q > C.mQ0 && rho * mem.sig > C.mrho) || ctrl.noev;           % evidence contradicts: rho*sig > 0 when hypothesis wrong
        V = 0.5 * (C.mw1 * chi1^2 + chi2^2);
        phi = C.mbeta * mem.Vk * exp(-C.mlam * (m.t - mem.tk)) + C.meps * (mem.k + 1);
        trig = 0;
        if ctrl.noenv                        % evidence alone decides (no envelope, no backup)
            fire = m.t - mem.tsw >= C.mdwell && contra;
        else
            fire = m.t - mem.tsw >= C.mdwell && ((V > phi && contra) || V > C.mbeta2 * phi);
        end
        if fire
            if ctrl.noenv || (V > phi && contra), trig = 1; else, trig = 2; end   % 1 evidence, 2 hard backup
            mem.sig = -mem.sig; mem.k = mem.k + 1; mem.tk = m.t; mem.tsw = m.t; mem.Vk = V;
            if C.mreset, mem.th = [0; 0]; end
        end
        info.trig = trig;
        info.rho = rho; info.V = V; info.phi = phi; info.sig = mem.sig;
        info.Q = Q; info.y = y; info.r = r;
        N = mem.sig * C.Nbar; ds = 0; mem.s = mem.k;
    case 'nominal'
        % Fixed nominal sign, as if the AGV had been hand-calibrated. Uses the
        % nominal hypothesis ctrl.bhat (= +1) and never adapts it; this must
        % NOT see m.b_true.
        N = -ctrl.bhat; ds = 0;
    case 'oracle'
        % True sign known, including the mid-run flip of S7.
        if isfield(m, 'b_true'), N = -m.b_true; else, N = -ctrl.bhat; end
        ds = 0;
    otherwise
        error('ctrl_bcn: unknown nuss mode ''%s''', ctrl.nuss);
end
u = N * nu2 / (a2 * v / P.L);

%% ---------------- updates --------------------------------------------------
info.s = mem.s; info.N = N; info.th = mem.th; info.a1 = alpha1; info.F2 = F2;
info.aclip = abs(nu1) > C.abar; info.q2clip = q2clip;
dsk = dt * ds; info.sclip = dsk > C.dsmax; mem.s = mem.s + min(dsk, C.dsmax);   % sampled-data guard
mem.th = min(max(mem.th + dt * [dth1; dth2], 0), C.thmax);   % projection onto [0, thmax]
end
