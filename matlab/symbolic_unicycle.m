syms ua ud real

syms pox poy real
syms vox voy real

syms x y phi v omega real

syms r real positive

syms phx phy real
vhx = v * cos(phi);
vhy = v * sin(phi);
ahx = ua * cos(phi) - v * ud * sin(phi);
ahy = ua * sin(phi) + v * ud * cos(phi);

po = [pox; poy];
vo = [vox; voy];

ph = [phx; phy];
vh = [vhx; vhy];
ah = [ahx; ahy];

p = po - ph;
v = vo - vh;
a = -ah;

h_prim = v'*v + p'*a + v'*a*sqrt(norm(p)^2 - r^2)/norm(v)^2 ...
    + p'*v*norm(v)^2/sqrt(norm(p)^2 - r^2);
h_prim = simplify(h_prim);


% ua, ud independent
C = subs(h_prim, [ua ud], [0 0]);
% Linear to ua
A_part = simplify(subs(h_prim, ud, 0) - C);
% Linear to ud
D_part = simplify(subs(h_prim, ua, 0) - C);
% check
check = simplify(C + A_part + D_part - h_prim);  % Should be 0
