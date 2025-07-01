%% Read data from rosbag file
bag = ros2bag('/home/maciej/Documents/repos/robust_c3cbf/rosbag2_recording');

robot_1_msgs = select(bag, "Topic", "/robot_1/robot_pose");
robot_1_msgs = readMessages(robot_1_msgs);
robot_2_msgs = select(bag, "Topic", "/robot_2/robot_pose");
robot_2_msgs = readMessages(robot_2_msgs);
robot_1_v_msgs = select(bag, "Topic", "/robot_1/v");
robot_1_v_msgs = readMessages(robot_1_v_msgs);
robot_2_v_msgs = select(bag, "Topic", "/robot_2/v");
robot_2_v_msgs = readMessages(robot_2_v_msgs);
robot_1_omega_msgs = select(bag, "Topic", "/robot_1/omega");
robot_1_omega_msgs = readMessages(robot_1_omega_msgs);
robot_2_omega_msgs = select(bag, "Topic", "/robot_2/omega");
robot_2_omega_msgs = readMessages(robot_2_omega_msgs);
robot_1_h_msgs = select(bag, "Topic", "/robot_1/h");
robot_1_h_msgs = readMessages(robot_1_h_msgs);
robot_2_h_msgs = select(bag, "Topic", "/robot_2/h");
robot_2_h_msgs = readMessages(robot_2_h_msgs);


robot_1_x = arrayfun(@(m) m{1}.pose.position.x, robot_1_msgs);
robot_1_y = arrayfun(@(m) m{1}.pose.position.y, robot_1_msgs);
robot_2_x = arrayfun(@(m) m{1}.pose.position.x, robot_2_msgs);
robot_2_y = arrayfun(@(m) m{1}.pose.position.y, robot_2_msgs);
robot_1_v = arrayfun(@(m) m{1}.data, robot_1_v_msgs);
robot_2_v = arrayfun(@(m) m{1}.data, robot_2_v_msgs);
robot_1_omega = arrayfun(@(m) m{1}.data, robot_1_omega_msgs);
robot_2_omega = arrayfun(@(m) m{1}.data, robot_2_omega_msgs);
robot_1_h = arrayfun(@(m) m{1}.data, robot_1_h_msgs);
robot_2_h = arrayfun(@(m) m{1}.data, robot_2_h_msgs);


%% Make plots
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

filename = 'robot_trajectory.gif';

for index = 1:5:length(robot_1_x)
    % draw robots
    p1 = nsidedpoly(100, 'Center', [robot_1_x(index) robot_1_y(index)], 'Radius', 0.5);
    h1 = plot(p1, 'FaceColor', 'r');
    p2 = nsidedpoly(100, 'Center', [robot_2_x(index) robot_2_y(index)], 'Radius', 0.5);
    h2 = plot(p2, 'FaceColor', 'b');

    drawnow
    
    % Makesnapshot for gif
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if index == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.5);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.5);
    end
    
    % remove robots drawing
    delete(h1)
    delete(h2)
end

distance = sqrt((robot_1_x(1:end)-robot_2_x(1:end)).^2 + (robot_1_y(1:end)-robot_2_y(1:end)).^2);
figure;
grid on;
hold on;
plot(distance);

figure;
subplot(2, 1, 1);
grid on;
hold on;
plot(robot_1_x, 'r');
plot(robot_2_x, 'b');
plot(zeros(size(robot_1_x)), 'r:', 'LineWidth',2)
plot(5*ones(size(robot_2_x)), 'b:', 'LineWidth',2)
subplot(2, 1, 2);
grid on;
hold on;
plot(robot_1_y, 'r');
plot(robot_2_y, 'b');
plot(10*ones(size(robot_1_y)), 'r:', 'LineWidth',2)
plot(5*ones(size(robot_2_y)), 'b:', 'LineWidth',2)