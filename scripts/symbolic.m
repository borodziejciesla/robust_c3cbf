syms x_o y_o v_xo v_yo real
syms x_r y_r theta_r real
syms v omega real
syms r real positive

p_rel = [x_o - x_r; y_o - y_r];
v_rel = [v_xo - v*cos(theta_r); v_yo - v*sin(theta_r)];
a_rel = [v*omega*sin(theta_r); -v*omega*cos(theta_r)];

h = p_rel'*v_rel + norm(v_rel)*sqrt(p_rel'*p_rel - r^2);
h_prim = v_rel'*v_rel + p_rel'*a_rel + v_rel'*a_rel*sqrt(p_rel'*p_rel-r^2)/(norm(v)) + p_rel'*v_rel*(norm(v))/sqrt(p_rel'*p_rel - r^2);

% define p, v, and a
syms p1 p2 v1 v2 a1 a2 real
p_ = [p1; p2];
v_ = [v1; v2];
a_ = [a1; a2];

% Calculate Jacobian matrix of Control Barrier Function
h_tmp = p_.' * v_ + norm(v_) * sqrt(p_.'*p_ - r^2);

J_p = jacobian(h_tmp, p_).';
J_v = jacobian(h_tmp, v_).';

J_p_actual = subs(J_p, [p1; p2], p_rel);
J_v_actual = subs(J_v, [v1; v2], v_rel);

% Calculate Jacobian matrix of Control Barrier Function derivative
h_prim_tmp = v_'*v_ + p_'*a_ + v_'*a_*sqrt(p_'*p_-r^2)/(norm(v_)) + p_'*v_*(norm(v_))/sqrt(p_'*p_ - r^2);

J_prim_p = jacobian(h_prim_tmp, p).';
J_prim_v = jacobian(h_prim_tmp, v_).';
J_prim_a = jacobian(h_prim_tmp, a_).';

J_prim_p_actual = subs(J_prim_p, [p1; p2], p_rel);
J_prim_v_actual = subs(J_prim_v, [v1; v2], v_rel);
J_prim_a_actual = subs(J_prim_a, [v1; v2], v_rel);
