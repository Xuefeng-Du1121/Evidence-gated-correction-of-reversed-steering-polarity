function ctrl = tii_controller(name, b_true)
% TII_CONTROLLER  Controller registry for this release: the proposed BCN
% controller and the reference/ablation variants that share its code.
% b_true is used ONLY by the oracle variant; all others start from the
% nominal polarity +1.
if nargin < 2, b_true = +1; end
ctrl.name = name; ctrl.bhat = +1;
ctrl.barrier = true; ctrl.dynF2 = true; ctrl.nuss = 'search'; ctrl.noev = false; ctrl.noenv = false;
ctrl.fn = @ctrl_bcn;
switch name
    case 'BCN'                       % proposed: barrier coords + evidence-gated polarity monitor
        ctrl.nuss = 'monitor';
    case 'BCN-noEv'                  % ablation: monitor without polarity evidence
        ctrl.nuss = 'monitor'; ctrl.noev = true;
    case 'BCN-noF2'                  % ablation: static heading boundary
        ctrl.nuss = 'monitor'; ctrl.dynF2 = false;
    case 'BCN-noBC'                  % ablation: no barrier coordinates
        ctrl.nuss = 'monitor'; ctrl.barrier = false; ctrl.dynF2 = false;
    case 'BCN-Nuss'                  % ablation: monitor replaced by a type-B Nussbaum search
    case 'BCN-nominal'               % no polarity handling, nominal sign +1
        ctrl.nuss = 'nominal';
    case 'BC-oracle'                 % reference: true sign known
        ctrl.nuss = 'oracle'; ctrl.bhat = b_true;
    otherwise
        error('tii_controller: %s is not part of this release', name);
end
end
