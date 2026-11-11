function [ym, t] = worst_recovery(P, rate_scale, v0)
% WORST_RECOVERY  Minimum-time recovery from a wrong hypothesis at full lock
% (lower bound on the worst-case lateral excursion, Section 5.4): wrong lock
% for the evidence window W, steering reversal at the rate limit, opposite
% lock until the heading error is back at zero. Actuator lag is ignored,
% which can only shorten the excursion. Returns the excursion ym (m) and the
% duration t (s). Used by make_tables (macros) and make_figures (fig. 6).
dr = P.drate * rate_scale; h = 1e-4; t = 0; psi = 0; y = 0; ym = 0;
while true
    if t < P.C.mwin, d = P.dmax; else, d = max(P.dmax - dr * (t - P.C.mwin), -P.dmax); end
    psi = psi + h * v0 / P.L * tan(d); y = y + h * v0 * sin(psi); ym = max(ym, y); t = t + h;
    if t > P.C.mwin && psi <= 0, break; end
end
end
