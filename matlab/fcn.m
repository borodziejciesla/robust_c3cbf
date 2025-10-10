function [ua_safe, ud_safe, h] = fcn(u, obs, X, do_filter)
if do_filter    
    % Parameters
    r = 1;
    alpha = 10;
    eps_safe = 1e-8;
    % control (nominal)
    ua = u(1);
    ud = u(2);
    % Obstacle
    pox = obs(1); poy = obs(2);
    vox = obs(3); voy = obs(4);
    % Host state
    xh = X(1); yh = X(2); phi = X(3); v = X(4);
    % positions / velocities / accelerations
    phx = xh; phy = yh;
    vhx = v * cos(phi); vhy = v * sin(phi);
    ahx = ua * cos(phi) - v * ud * sin(phi);
    ahy = ua * sin(phi) + v * ud * cos(phi);
    % relative
    prx = pox - phx;
    pry = poy - phy;
    vrx = vox - vhx;
    vry = voy - vhy;

    po = [pox; poy]; vo = [vox; voy];
    ph = [phx; phy]; vh = [vhx; vhy]; ah = [ahx; ahy];

    pr = po - ph; vr = vo - vh; ar = -ah;

    % bezpieczne denominatory
    vel_rel_sq = (vrx)^2 + (vry)^2;
    vel_rel_sq = max(vel_rel_sq, eps_safe);

    dist_sq = prx^2 + pry^2;
    dist = sqrt(max(dist_sq, eps_safe));
    inside_sqrt = dist_sq - r^2;
    inside_sqrt = max(inside_sqrt, 0);

    % Base h
    h = pr'*vr + norm(pr)*norm(vr)*sqrt(norm(pr)^2 - r^2)/norm(pr);
    % skrócone flagi do wyrażeń
    S = sqrt(inside_sqrt);
    V2 = vel_rel_sq;

    % Wyprowadzone części (numerycznie stabilne)
    % UWAGA: upewnij się, że te formuły odpowiadają Twojemu symbolicznemu h'
    a_part = (cos(phi)*(phx - pox) + sin(phi)*(phy - poy) - ((cos(phi)*(vox - v*cos(phi)) + sin(phi)*(voy - v*sin(phi)))*(abs(phx - pox)^2 - r^2 + abs(phy - poy)^2)^(1/2))/(abs(vox - v*cos(phi))^2 + abs(voy - v*sin(phi))^2));
    d_part = (((v*sin(phi)*(vox - v*cos(phi)) - v*cos(phi)*(voy - v*sin(phi)))*(abs(phx - pox)^2 - r^2 + abs(phy - poy)^2)^(1/2))/(abs(vox - v*cos(phi))^2 + abs(voy - v*sin(phi))^2) + v*cos(phi)*(phy - poy) - v*sin(phi)*(phx - pox));
    free_part = (vox - v*cos(phi))^2 + (voy - v*sin(phi))^2 - (((vox - v*cos(phi))*(phx - pox) + (voy - v*sin(phi))*(phy - poy))*(abs(vox - v*cos(phi))^2 + abs(voy - v*sin(phi))^2))/(- r^2 + abs(phx - pox)^2 + abs(phy - poy)^2)^(1/2);

    % Macierze do QP: A*u <= b
    A = [a_part, d_part];
    b = -free_part - alpha * h;

    % QP: min 1/2 (u-u_nom)'*H*(u-u_nom)  => H, f = -H*u_nom
    w_a = 1; w_d = 1;
    H = diag([w_a, w_d]);
    f = -2 * H * u;

    % For robust
    delta = sqrt(norm(pr)^2 - r^2);

    nabla_p_h = vr + (norm(vr) / delta)*pr;
    nabla_v_h = pr + (delta / norm(vr))*vr;

    nabla_p_h_prim = ar + (vr'*ar / norm(vr) / delta)*pr + (norm(vr) / delta)*vr - (norm(vr)*pr'*vr/delta^3)*pr;
    nabla_v_h_prim = 2*vr + delta * (ar/norm(vr) - (vr'*ar/norm(vr)^3)*vr) + (norm(vr)*pr + (pr'*vr/norm(vr))*vr) / delta;
    nabla_a_h_prim = pr + (delta/norm(vr))*vr;

    cov_p = 0.2^2;
    cov_v = 0.2^2;
    cov_a = 0.2^2;

    nabla_p = nabla_p_h_prim + nabla_p_h;
    nabla_v = nabla_v_h_prim + nabla_v_h;
    nabla_a = nabla_a_h_prim;

    cov_c = nabla_p'*cov_p*nabla_p +  nabla_v'*cov_v*nabla_v + nabla_a'*cov_a*nabla_a;
    sigma_c = sqrt(cov_c);

    b = b + norminv(0.95) * sigma_c;

    % bounds (przykładowe, dostosuj do fizyki)
    lb = [];
    ub = [];

    options = optimoptions('quadprog', ...
        'Algorithm', 'active-set', ...
        'Display', 'off', ...
        'ConstraintTolerance', 1e-6, ...
        'OptimalityTolerance', 1e-6);

    % solve with fallback
    u_safe = quadprog(H, f, -A, -b, [], [], lb, ub, u, options);
    if isempty(u_safe) || any(isnan(u_safe))
        u_safe = u;
    end

    ua_safe = u_safe(1);
    ud_safe = u_safe(2);
else
    ua_safe = u(1);
    ud_safe = u(2);
    h = 0;
end
end