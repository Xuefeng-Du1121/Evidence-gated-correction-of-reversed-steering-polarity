function k = tii_kappa(sc, s)
% Path curvature as a function of arc length (1/m).
k = sc.ka * sin(sc.kw * s + sc.kph);
end
