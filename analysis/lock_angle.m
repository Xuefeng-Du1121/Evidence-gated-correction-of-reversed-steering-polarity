function [dlo, Tlock] = lock_angle(P, rate_scale, tau_scale)
% LOCK_ANGLE  Lock threshold and the earliest time to reach it (Theorem 1,
% Remark 2, Lemma 3 at lock). Under (1c), delta-dot = sat((u - delta)/tau) with
% |u| <= dmax, a full-lock command drives delta up at the rate limit until
% u - delta < tau * drate and then exponentially towards dmax, which it never
% reaches. Lock is therefore counted at dlo = 0.9 dmax, which is reached in
% finite time. Returns dlo (rad) and the time Tlock (s) to reach it from a
% centred wheel under a constant full-lock command, at the given actuator
% scaling (rate_scale, tau_scale default 1).
if nargin < 2, rate_scale = 1; end
if nargin < 3, tau_scale = 1; end
dlo = 0.9 * P.dmax;
dr = P.drate * rate_scale; tau = P.tau * tau_scale;
db = tau * dr;                                  % below dmax - db the rate limit binds
if P.dmax - dlo >= db                           % reached during the rate-limited ramp
    Tlock = dlo / dr;
else
    t1 = (P.dmax - db) / dr;                    % end of the ramp
    Tlock = t1 + tau * log(db / (P.dmax - dlo));   % dmax - db e^{-(t - t1)/tau} = dlo
end
end
