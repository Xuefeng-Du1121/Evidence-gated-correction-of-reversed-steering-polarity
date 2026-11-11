function w = window_min(P, Qminus, v)
% WINDOW_MIN  Window rule of Section 5.3: the shortest evidence window whose
% excitation at the lock angle clears Q- at speed v,
% W_min = L sqrt(Q-) / (v tan(dlo)), dlo = lock_angle(P).
% Single source for the tables (EvWmin, Wmap*Ratio) and for the abscissa of
% fig14, so the figure cannot drift from the text again.
w = P.L * sqrt(Qminus) / (v * tan(lock_angle(P)));
end
