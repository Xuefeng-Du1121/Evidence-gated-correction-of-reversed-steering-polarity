function [F1, co] = tii_corridor(sc, t)
% TII_CORRIDOR  Admissible lateral band |e_y - c_o(t)| < F1(t).
% Obstacles intrude from one side: the band shrinks by h/2 and shifts away
% from the obstacle by h/2, so the free edge on the far side is unchanged.

F1 = sc.W0 + sc.Wa * sin(sc.Ww * t + sc.Wph);
co = 0;
for k = 1:numel(sc.ev)
    e = sc.ev(k);
    g = 0.5 * (tanh(2 * (t - e.t0) / e.tr) - tanh(2 * (t - e.t0 - e.dur) / e.tr));
    F1 = F1 - 0.5 * e.h * g;
    co = co - 0.5 * e.h * e.side * g;
end
if ~isempty(sc.fork)
    f = sc.fork;
    co = co + f.A * sin(f.w * t + f.ph);
    F1 = F1 - f.Wa * (0.5 + 0.5 * sin(f.w * t + f.ph + pi / 2));
end
end
