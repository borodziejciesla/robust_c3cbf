alphas = [1];
outputs = [];

do_filter = false;

for index = 1:length(alphas)
    alpha = alphas(index);
    outputs = [outputs; sim("model.slx")];
end

trajectory_figure = figure();
distance_figure = figure();
control_acceleration_figure = figure();
control_yaw_rate_figure = figure();
position_figure = figure();

for alpha_index = 1:length(outputs)
    out = outputs(alpha_index);

    figure(trajectory_figure);
    title("Trajectories");
    grid on; hold on;
    axis equal;
    xlabel("x [m]");
    ylabel("y [m]");
    legend("show");
    plot(out.logsout.find('x1').Values.Data, out.logsout.find('y1').Values.Data, ...
        "DisplayName", "Robot 1 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('x2').Values.Data, out.logsout.find('y2').Values.Data, ...
        "DisplayName", "Robot 2 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    
    figure(distance_figure);
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
        "DisplayName", "Distance between robots - \alpha = " + num2str(alphas(alpha_index)));
    plot(out.logsout.find('x1').Values.Time, ones(size(out.logsout.find('x1').Values.Time)), ...
        "k--", ...
        "LineWidth", 2, ...
        "DisplayName", "Minimum safe distance");
    
    figure(control_acceleration_figure);
    subplot(2, 1, 1);
    title("Robot 1 Acceleration");
    grid on; hold on;
    xlabel("Time [s]");
    ylabel("Acceleration [m/s^2]");
    legend("show");
    plot(out.logsout.find('a1').Values.Time, out.logsout.find('a1').Values.Data, ...
        "DisplayName", "a_1 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('a_safe_1').Values.Time, out.logsout.find('a_safe_1').Values.Data, ...
        "DisplayName", "a_{1, safe} - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    subplot(2, 1, 2);
    xlabel("Time [s]");
    ylabel("Acceleration [m/s^2]");
    title("Robot 2 Acceleration");
    legend("show");
    grid on; hold on;
    plot(out.logsout.find('a2').Values.Time, out.logsout.find('a2').Values.Data, ...
        "DisplayName", "a_2 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('a_safe_2').Values.Time, out.logsout.find('a_safe_2').Values.Data, ...
        "DisplayName", "a_{2, safe} - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    
    figure(control_yaw_rate_figure);
    subplot(2, 1, 1);
    title("Robot 1 Yaw Rate");
    grid on; hold on;
    xlabel("Time [s]");
    ylabel("Yaw Rate [rad/s]");
    legend("show");
    plot(out.logsout.find('d1').Values.Time, out.logsout.find('d1').Values.Data, ...
        "DisplayName", "d_1 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('d_safe_1').Values.Time, out.logsout.find('d_safe_1').Values.Data, ...
        "DisplayName", "d_{1, safe} - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    subplot(2, 1, 2);
    title("Robot 2 Yaw Rate");
    grid on; hold on;
    legend("show");
    xlabel("Time [s]");
    ylabel("Yaw Rate [rad/s]");
    plot(out.logsout.find('d2').Values.Time, out.logsout.find('d2').Values.Data, ...
        "DisplayName", "d_2 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('d_safe_2').Values.Time, out.logsout.find('d_safe_2').Values.Data, ...
        "DisplayName", "d_{2, safe} - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);

    figure(position_figure);
    subplot(2, 1, 1);
    grid on; hold on;
    xlabel("Time [s]");
    ylabel("x [m]");
    legend("show");
    plot(out.logsout.find('x1').Values.Time, out.logsout.find('x1').Values.Data, ...
        "DisplayName", "Robot 1 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('x2').Values.Time, out.logsout.find('x2').Values.Data, ...
        "DisplayName", "Robot 2 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    subplot(2, 1, 2);
    grid on; hold on;
    xlabel("Time [s]");
    ylabel("y [m]");
    legend("show");
    plot(out.logsout.find('y1').Values.Time, out.logsout.find('y1').Values.Data, ...
        "DisplayName", "Robot 1 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 2);
    plot(out.logsout.find('y2').Values.Time, out.logsout.find('y2').Values.Data, ...
        "DisplayName", "Robot 2 - \alpha = " + num2str(alphas(alpha_index)), ...
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
    xlim([-12 12]);
    ylim([-12 12]);
    axis equal;
    plot(robot_1_x, robot_1_y, 'r', 'LineWidth', 2, 'DisplayName', "Robot 1 trajectory");
    plot(robot_2_x, robot_2_y, 'b', 'LineWidth', 2, 'DisplayName', "Robot 2 trajectory");
    xlabel("x [m]")
    ylabel("y [m]")
    legend("show");
    title("\alpha = " + num2str(alphas(alpha_index)));

    filename = "robot_trajectory_alpha"+num2str(alphas(alpha_index))+".gif";

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
end
