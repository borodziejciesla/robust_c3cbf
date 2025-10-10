figure();
title("Trajectories");
grid on; hold on;
axis equal;
xlabel("x [m]");
ylabel("y [m]");
legend("show");
plot(out.logsout.find('x1').Values.Data, out.logsout.find('y1').Values.Data, ...
    "DisplayName", "Robot 1", ...
    "LineWidth", 2);
plot(out.logsout.find('x2').Values.Data, out.logsout.find('y2').Values.Data, ...
    "DisplayName", "Robot 2", ...
    "LineWidth", 2);

figure();
title("Distance between robots");
grid on; hold on;
xlabel("Time [s]");
ylabel("Distance [m]");
legend("show");
x_diff = out.logsout.find('x1').Values.Data - out.logsout.find('x2').Values.Data;
y_diff = out.logsout.find('y1').Values.Data - out.logsout.find('y2').Values.Data;
dist = sqrt(x_diff.^2 + y_diff.^2);
plot(out.logsout.find('x1').Values.Time, dist, ...
    "LineWidth", 2, ...
    "DisplayName", "Distance between robots");
plot(out.logsout.find('x1').Values.Time, ones(size(out.logsout.find('x1').Values.Time)), ...
    "k--", ...
    "LineWidth", 2, ...
    "DisplayName", "Minimum safe distance");

figure();
subplot(2, 1, 1);
title("Robot 1 Acceleration");
grid on; hold on;
xlabel("Time [s]");
ylabel("Acceleration [m/s^2]");
legend("show");
plot(out.logsout.find('a1').Values.Time, out.logsout.find('a1').Values.Data, ...
    "DisplayName", "a_1", ...
    "LineWidth", 2);
plot(out.logsout.find('a_safe_1').Values.Time, out.logsout.find('a_safe_1').Values.Data, ...
    "DisplayName", "a_{1, safe}", ...
    "LineWidth", 2);
subplot(2, 1, 2);
xlabel("Time [s]");
ylabel("Acceleration [m/s^2]");
title("Robot 2 Acceleration");
legend("show");
grid on; hold on;
plot(out.logsout.find('a2').Values.Time, out.logsout.find('a2').Values.Data, ...
    "DisplayName", "a_1", ...
    "LineWidth", 2);
plot(out.logsout.find('a_safe_2').Values.Time, out.logsout.find('a_safe_2').Values.Data, ...
    "DisplayName", "a_{2, safe}", ...
    "LineWidth", 2);

figure();
subplot(2, 1, 1);
title("Robot 1 Yaw Rate");
grid on; hold on;
xlabel("Time [s]");
ylabel("Yaw Rate [rad/s]");
legend("show");
plot(out.logsout.find('d1').Values.Time, out.logsout.find('d1').Values.Data, ...
    "DisplayName", "d_1", ...
    "LineWidth", 2);
plot(out.logsout.find('d_safe_1').Values.Time, out.logsout.find('d_safe_1').Values.Data, ...
    "DisplayName", "d_{1, safe}", ...
    "LineWidth", 2);
subplot(2, 1, 2);
title("Robot 2 Yaw Rate");
grid on; hold on;
legend("show");
xlabel("Time [s]");
ylabel("Yaw Rate [rad/s]");
plot(out.logsout.find('d2').Values.Time, out.logsout.find('d2').Values.Data, ...
    "DisplayName", "d_1", ...
    "LineWidth", 2);
plot(out.logsout.find('d_safe_2').Values.Time, out.logsout.find('d_safe_2').Values.Data, ...
    "DisplayName", "d_{2, safe}", ...
    "LineWidth", 2);

%% Make plots
robot_1_x = out.logsout.find('x1').Values.Data;
robot_1_y = out.logsout.find('y1').Values.Data;
robot_2_x = out.logsout.find('x2').Values.Data;
robot_2_y = out.logsout.find('y2').Values.Data;
theta_1   = out.logsout.find('yaw_1').Values.Data; % <-- heading robota 1 [rad]
theta_2   = out.logsout.find('yaw_2').Values.Data; % <-- heading robota 2 [rad]

figure;
grid on;
hold on;
xlim([-7 7]);
ylim([-2 12]);
axis equal;
plot(robot_1_x, robot_1_y, 'r', 'LineWidth', 2, 'DisplayName', "Robot 1 trajectory");
plot(robot_2_x, robot_2_y, 'b', 'LineWidth', 2, 'DisplayName', "Robot 2 trajectory");
xlabel("x [m]")
ylabel("y [m]")
legend("show");

filename = 'robot_trajectory.gif';

heading_len = 1.0; % długość strzałki reprezentującej orientację

for index = 1:5:length(robot_1_x)
    % --- Robot 1 ---
    p1 = nsidedpoly(100, 'Center', [robot_1_x(index) robot_1_y(index)], 'Radius', 0.5);
    h1 = plot(p1, 'FaceColor', 'r');
    
    % Rysowanie headingu robota 1
    hx1 = [robot_1_x(index), robot_1_x(index) + heading_len*cos(theta_1(index))];
    hy1 = [robot_1_y(index), robot_1_y(index) + heading_len*sin(theta_1(index))];
    h_head1 = plot(hx1, hy1, 'k-', 'LineWidth', 2); % czarna linia kierunku

    % --- Robot 2 ---
    p2 = nsidedpoly(100, 'Center', [robot_2_x(index) robot_2_y(index)], 'Radius', 0.5);
    h2 = plot(p2, 'FaceColor', 'b');
    
    % Rysowanie headingu robota 2
    hx2 = [robot_2_x(index), robot_2_x(index) + heading_len*cos(theta_2(index))];
    hy2 = [robot_2_y(index), robot_2_y(index) + heading_len*sin(theta_2(index))];
    h_head2 = plot(hx2, hy2, 'k-', 'LineWidth', 2);

    drawnow;
    
    % --- Tworzenie klatki GIF ---
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if index == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    
    % --- Usuwanie obiektów przed kolejną klatką ---
    delete(h1)
    delete(h2)
    delete(h_head1)
    delete(h_head2)
end
