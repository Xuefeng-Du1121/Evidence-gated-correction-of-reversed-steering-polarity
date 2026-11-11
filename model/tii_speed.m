function v = tii_speed(sc, t)
v = sc.v0 + sc.va * sin(sc.vw * t);
end
