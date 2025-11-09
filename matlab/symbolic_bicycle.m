syms ua ud real

syms pox poy real
syms vox voy real

syms x y phi v real
syms L real positive

syms r real positive

syms phx phy real

vhx = v * cos(phi) ;
vhy = v * sin(phi);
ahx = ua * cos(phi) - (v^2/L)*ud*sin(phi);
ahy = ua * sin(phi) + (v^2/L)*ud*cos(phi);

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

%% Base control
syms x y theta v real
syms x_d y_d theta_d v_d real
syms ua ud real
syms L real positive

X = [x;y;theta;v];
X_d = [x_d;y_d;theta_d;v_d];
Z = X-X_d;

x_p = v * cos(theta);
y_p = v * sin(theta);
theta_p = v * tan(ud) / L;
v_p = ua;

f = [x_p;y_p;theta_p;v_p];
V = 0.5 * (x^2 + y^2 + theta^2 + v^2);

dV = [diff(V, x)'; diff(V, y); diff(V, theta); diff(V, v)];

lfv = dV'*f;

C = subs(lfv, [ua ud], [0 0]);
% Linear to ua
A_part = simplify(subs(lfv, ud, 0) - C);
% Linear to ud
D_part = simplify(subs(lfv, ua, 0) - C);
% check
check = simplify(C + A_part + D_part - lfv);  % Should be 0