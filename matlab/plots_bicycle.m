dt = 0.01;
T = 15;

alphas = [1, 5, 10];
outputs = [];

do_filter_robot_1 = true;
do_filter_robot_2 = true;
do_probabilistic_filter = true;

for index = 1:length(alphas)
    alpha = alphas(index);
    outputs = [outputs; sim("model_bicycle.slx")];
end

%% 
trajectory_figure = figure();
distance_figure = figure();
control_acceleration_figure = figure();
control_yaw_rate_figure = figure();
position_figure = figure();

c = ['r', 'b', 'g', 'k', 'm'];

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
        "Color", c(alpha_index), ...
        "DisplayName", "Robot 1 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 3);
    plot(out.logsout.find('x2').Values.Data, out.logsout.find('y2').Values.Data, ...
        "--", ...
        "Color", c(alpha_index), ...
        "DisplayName", "Robot 2 - \alpha = " + num2str(alphas(alpha_index)), ...
        "LineWidth", 3);
    
    figure(distance_figure);
    title("Distance between robots");
    grid on; hold on;
    xlabel("Time [s]");
    ylabel("Distance [m]");
    legend("show");
    x_diff = out.logsout.find('x1').Values.Data - out.logsout.find('x2').Values.Data;
    y_diff = out.logsout.find('y1').Values.Data - out.logsout.find('y2').Values.Data;
    dist = sqrt(x_diff.^2 + y_diff.^2);
    plot(out.logsout.find('x1').Values.Time, ones(size(out.logsout.find('x1').Values.Time)), ...
        "k--", ...
        "LineWidth", 2, ...
        "DisplayName", "Minimum safe distance");
    plot(out.logsout.find('x1').Values.Time, dist, ...
        "LineWidth", 2, ...
        "DisplayName", "Distance between robots - \alpha = " + num2str(alphas(alpha_index)));
        
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

    % continue;
    
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

    car_length = 1.0;     % długość prostokąta robota [m]
    car_width  = 0.5;     % szerokość prostokąta robota [m]

    for index = 1:50:length(robot_1_x)
        % --- Robot 1 ---
        x_c1 = robot_1_x(index);
        y_c1 = robot_1_y(index);
        yaw1 = theta_1(index);
    
        % Współrzędne narożników prostokąta w lokalnym układzie (środek w (0,0))
        rect_local = [ car_length/2,  car_width/2;
                       car_length/2, -car_width/2;
                      -car_length/2, -car_width/2;
                      -car_length/2,  car_width/2;
                       car_length/2,  car_width/2]';  % zamykamy kształt
    
        % Macierz obrotu
        R = [cos(yaw1), -sin(yaw1);
             sin(yaw1),  cos(yaw1)];
    
        % Obrót i przesunięcie prostokąta
        rect_global1 = R * rect_local + [x_c1; y_c1];
    
        % Rysowanie prostokąta
        h1 = fill(rect_global1(1, :), rect_global1(2, :), 'r', 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    
        % Rysowanie headingu
        hx1 = [x_c1, x_c1 + heading_len*cos(yaw1)];
        hy1 = [y_c1, y_c1 + heading_len*sin(yaw1)];
        h_head1 = plot(hx1, hy1, 'k-', 'LineWidth', 2);
    
        % --- Robot 2 ---
        x_c2 = robot_2_x(index);
        y_c2 = robot_2_y(index);
        yaw2 = theta_2(index);
    
        rect_global2 = R * rect_local + [x_c2; y_c2]; % możesz użyć nowego R dla yaw2
        R2 = [cos(yaw2), -sin(yaw2);
              sin(yaw2),  cos(yaw2)];
        rect_global2 = R2 * rect_local + [x_c2; y_c2];
        h2 = fill(rect_global2(1, :), rect_global2(2, :), 'b', 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    
        hx2 = [x_c2, x_c2 + heading_len*cos(yaw2)];
        hy2 = [y_c2, y_c2 + heading_len*sin(yaw2)];
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
