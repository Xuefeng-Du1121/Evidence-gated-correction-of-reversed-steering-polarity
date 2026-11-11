% DEMO_WRONG_POLARITY  The proposed BCN controller recovering from a wrong
% initial polarity hypothesis (scenario S2: start near the corridor edge,
% heading outward; true polarity b = -1, controller starts from +1), next to
% the same layers without polarity handling.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'model'), fullfile(here, '..', 'controller'), fullfile(here, '..', 'analysis'));

P = tii_default_params();
sc = tii_scenario('S2', -1, 1017);           % first main-study seed of the paper
R  = tii_simulate(sc, tii_controller('BCN'), P, true);
Rn = tii_simulate(sc, tii_controller('BCN-nominal'), P, true);

fprintf('BCN:         success %d, max corridor ratio %.3f, reversals %d, first at %.2f s\n', ...
    R.success, R.gmax, R.nsw, R.log.t(find(R.log.trig > 0, 1)));
fprintf('BCN-nominal: success %d (fixed sign +1)\n', Rn.success);
[dlo, Tl] = lock_angle(P);
fprintf('lock angle %.0f deg, reached no sooner than %.2f s from a centred wheel\n', rad2deg(dlo), Tl);

L = R.log; Ln = Rn.log;
figure('Color', 'w');
subplot(3, 1, 1); hold on;
plot(L.t, abs(L.ey - L.co) ./ L.F1, 'LineWidth', 1.1); plot(Ln.t, abs(Ln.ey - Ln.co) ./ Ln.F1, '--', 'LineWidth', 1.1);
yline(1, 'r-'); ylim([0 1.5]); xlim([0 5]); ylabel('\gamma'); legend('BCN', 'BCN-nominal', 'Location', 'best');
subplot(3, 1, 2); plot(L.t, rad2deg(L.delta), 'LineWidth', 1.1); xlim([0 5]); ylabel('\delta (deg)');
subplot(3, 1, 3); stairs(L.t, -L.sig, 'LineWidth', 1.1); hold on; yline(sc.b0, ':');
ylim([-1.5 1.5]); xlim([0 5]); ylabel('b-hat, b'); xlabel('Time (s)');
