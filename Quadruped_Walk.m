clear, clc, close all

disp('RBE521HW5Problem4')

%% Initalize Data

L1 = .5;
L2 = .8;

beta = 0.75;
% Cernter of grvity = COG
vel = 0.1; 
                  
L_stride = 1;% Stride length (displacement of the center of gravity (COG) during a cycle).
T = L_stride/vel; % Period of a cycle
T_tra = (1-beta)*T;% Duration of transfer phase
T_sta = beta*T;% Duration of stationary phase

%% Generate Gait

phi(1) = 0; % Kinematic phase leg 1
phi(2) = 0.5; % Kinematic phase leg 2
phi(3) = beta; % Kinematic phase leg 3
phi(4) = beta - phi(2); % Kinematic phase leg 4

%Check if p(i) is greater than or equal to 1
for leg = 1:4
    if phi(leg) >= 1
        phi(leg) = phi(leg) - 1
    end
end

%% Plan Positional Foot Trajectory

%Transfer duration time divided into por = 5 portions
por = 5;
T0 = 0;
T1 = T_tra/por;

T2 = 2*T1;
T3 = 3*T1;
T4 = 4*T1;

T5 = T_tra;
T6 = T5 + T1;
T7 = T5 + T1*2;
T8 = T5 + T1*3;
T9 = T5 + T1*4;
T10 = T5 + T1*5;
T11 = T5 + T1*6;
T12 = T5 + T1*7;
T13 = T5 + T1*8;
T14 = T5 + T1*9;
T15 = T5 + T1*10;
T16 = T5 + T1*11;
T17 = T5 + T1*12;
T18 = T5 + T1*13;
T19 = T5 + T1*14;
T20 = T5 + T1*15;
T21 = T5 + T1*16;
T22 = T5 + T1*17;
T23 = T5 + T1*18;

T_pack=[T0, T1, T2, T3, T4, T5, T6,  T7,  T8,  T9,  T10,  T11,  T12,  T13,  T14,  T15,  T16,  T17,  T18,  T19,  T20,  T21,  T22, T23];

% Based in ground cordinates
x_dot_max_foot =  L_stride/(2 * T1);% Max leg transfer speed
z_dot_max_foot = x_dot_max_foot;

% x_dot_foot = [0, 0, x_dot_max_foot, x_dot_max_foot, 0, 0];
x_dot_foot = [0, 0, x_dot_max_foot, x_dot_max_foot, 0, 0, 0, 0, x_dot_max_foot, x_dot_max_foot, 0, 0, 0, 0, x_dot_max_foot, x_dot_max_foot, 0, 0, 0, 0, x_dot_max_foot, x_dot_max_foot, 0, 0];




% vvv test copy
x_step = 0.75/19;
x_step2 = -0.01;
%x_step2 = 0.5;
for leg = 1:4
    x_foot(leg, 1) = -L_stride/2;
    x_foot(leg, 2) = x_foot(leg, 1);
    x_foot(leg, 3) = x_foot(leg, 2) + (T2-T1) * x_dot_max_foot/2;
    x_foot(leg, 4) = x_foot(leg, 3) + (T3-T2) * x_dot_max_foot;
    x_foot(leg, 5) = x_foot(leg, 4) + (T4-T3) * x_dot_max_foot/2;
    x_foot(leg, 6) = x_foot(leg, 5);

    %Horizonatal positions supporting
    x_foot(leg, 7) = x_foot(leg, 6) - x_step*4;% making this value equal to the last seems like its not actualy positioning the foot at the same point
    x_foot(leg, 8) = x_foot(leg, 7) - x_step2;
    x_foot(leg, 9) = x_foot(leg, 8) - x_step2;
    x_foot(leg, 10) = x_foot(leg, 9) - x_step2;
    x_foot(leg, 11) = x_foot(leg, 10) - x_step2;
    x_foot(leg, 12) = x_foot(leg, 11) - x_step2;
    x_foot(leg, 13) = x_foot(leg, 12) - x_step2;
    x_foot(leg, 14) = x_foot(leg, 13) - x_step2;
    x_foot(leg, 15) = x_foot(leg, 14) - x_step2;
    x_foot(leg, 16) = x_foot(leg, 15) - x_step2;
    x_foot(leg, 17) = x_foot(leg, 16) - x_step2;
    x_foot(leg, 18) = x_foot(leg, 17) - x_step2;
    x_foot(leg, 19) = x_foot(leg, 18) - x_step2;
    x_foot(leg, 20) = x_foot(leg, 19) - x_step2;
    x_foot(leg, 21) = x_foot(leg, 20) - x_step2;
    x_foot(leg, 22) = x_foot(leg, 21) - x_step2;
    x_foot(leg, 23) = x_foot(leg, 22) - x_step2;
    x_foot(leg, 24) = x_foot(leg, 23) - x_step2;

end



z_dot_foot = [0, z_dot_max_foot, 0, 0, -z_dot_max_foot, 0, 0, z_dot_max_foot, 0, 0, -z_dot_max_foot, 0, 0, z_dot_max_foot, 0, 0, -z_dot_max_foot, 0, 0, z_dot_max_foot, 0, 0, -z_dot_max_foot, 0];

% Vertical positions
for leg = 1:4
    z_foot(leg, 1) = 0;
    z_foot(leg, 2) = z_foot(leg, 1) + (T1-T0) * z_dot_max_foot/2;
    z_foot(leg, 3) = z_foot(leg, 2) + (T2-T1) * z_dot_max_foot/2;
    z_foot(leg, 4) = z_foot(leg, 3);
    z_foot(leg, 5) = z_foot(leg, 4) - (T4-T3) * z_dot_max_foot/2;
    z_foot(leg, 6) = z_foot(leg, 5) - (T5-T4) * z_dot_max_foot/2;
end
%Vertical positions supporting
ground_h = -1;
for leg = 1:4
    z_foot(leg, 7) = ground_h;
    z_foot(leg, 8) = ground_h;
    z_foot(leg, 9) = ground_h;
    z_foot(leg, 10) = ground_h;
    z_foot(leg, 11) = ground_h;
    z_foot(leg, 12) = ground_h;
    z_foot(leg, 13) = ground_h;
    z_foot(leg, 14) = ground_h;
    z_foot(leg, 15) = ground_h;
    z_foot(leg, 16) = ground_h;
    z_foot(leg, 17) = ground_h;
    z_foot(leg, 18) = ground_h;
    z_foot(leg, 19) = ground_h;
    z_foot(leg, 20) = ground_h;
    z_foot(leg, 21) = ground_h;
    z_foot(leg, 22) = ground_h;
    z_foot(leg, 23) = ground_h;
    z_foot(leg, 24) = ground_h;
end


% Cooridinte systems where % The x axis of the ground and the hip frame are parallel
%frame b is a frame attached to the hip joint

H_robot = 1; % Robot height (meters)
D_hip_foot = L1 + L2; %Distance from hip to foot from top view (meters)

lim_H = [-30*pi/180, 30*pi/180, -90*pi/180, 90*pi/180, -150*pi/180, 150*pi/180];


% Position of hip (meters)
x_hip = zeros(4, 1);
for leg = 1:4
    x_hip(leg, 1) = - (1-beta) * L_stride/2;
end
z_hip = zeros(4, 1);
for leg = 1:4
    z_hip(leg, 1) = H_robot;
end
%x_hip

D_t = T_tra/5;
x_dot_hip = vel;
z_dot_hip = 0;

% disp("checker gama")
gama = 0;

for leg = 1:4
    for time = 1:5 
        x_hip(leg, time + 1) = x_hip(leg, time)+x_dot_hip * D_t;
        z_hip(leg, time + 1) = z_hip(leg, time)+z_dot_hip * D_t;
    end
    for time = 1:6
        x_foot_hip(leg, time) = x_foot(leg, time) - x_hip(leg, time);
        z_foot_hip(leg, time) = z_foot(leg, time) - z_hip(leg, time);
    end
end

%Supporting Phase values
x_hip(:,7) = zeros(4, 1);
for leg = 1:4
    x_hip(leg, 1) = - (1-beta) * L_stride/2;
end
z_hip(:,7) = zeros(4, 1);
for leg = 1:4
    z_hip(leg, 1) = H_robot;
end
for leg = 1:4
    for time = 7:23
        x_hip(leg, time + 1) = x_hip(leg, time)+x_dot_hip * D_t;%error with inpot 
        z_hip(leg, time + 1) = z_hip(leg, time)+z_dot_hip * D_t;
    end
    for time = 7:24
        x_foot_hip(leg, time) = x_foot(leg, time) - x_hip(leg, time);
        z_foot_hip(leg, time) = z_foot(leg, time) - z_hip(leg, time);
    end
end

%no errors



y_foot_hip = [D_hip_foot *sin(30 * pi/180), -D_hip_foot * sin(30 * pi/180), D_hip_foot,  -D_hip_foot,  D_hip_foot * sin(30 * pi/180),  -D_hip_foot * sin(30 * pi/180)];

for leg = 1:4
    for time = 1:6

        x_foot_H(leg, time) = [cos(lim_H(leg)),-sin(lim_H(leg)),0]*[x_foot_hip(leg, time);y_foot_hip(leg);z_foot_hip(leg, time)];
        y_foot_H(leg, time) = [sin(lim_H(leg)),cos(lim_H(leg)),0]*[x_foot_hip(leg, time);y_foot_hip(leg);z_foot_hip(leg, time)];
        z_foot_H(leg, time) = z_foot_hip(leg, time);

    end
end
%Supporting phase values
for leg = 1:4
    for time = 7:24

        x_foot_H(leg, time) = [cos(lim_H(leg)),-sin(lim_H(leg)),0]*[x_foot_hip(leg, time);y_foot_hip(leg);z_foot_hip(leg, time)];
        y_foot_H(leg, time) = [sin(lim_H(leg)),cos(lim_H(leg)),0]*[x_foot_hip(leg, time);y_foot_hip(leg);z_foot_hip(leg, time)];
        z_foot_H(leg, time) = z_foot_hip(leg, time);

    end
end


%% Plot Data

% subplot(3, 2, 1)
% plot(T_pack, x_dot_foot)
% subplot(3, 2, 2)
% plot(T_pack, z_dot_foot)
% subplot(3, 2, 1)
plot(T_pack, x_dot_foot)
xlabel('Time')
ylabel('Horizontal Foot Velocity')
subplot(3, 2, 2)
plot(T_pack, z_dot_foot)
xlabel('Time')
ylabel('Horizontal Foot Velocity')

subplot(3, 2, 3)
plot(T_pack, x_foot(4, :))
xlabel('Time')
ylabel('Horizontal Foot Position')
subplot(3, 2, 4)
plot(T_pack, z_foot(4, :))
xlabel('Time')
ylabel('Vertical Foot Position')

% subplot(3, 2, 5)
% plot(x_foot(4, :), z_foot(4, :))

% subplot(3, 2, 6)
% plot(x_foot_hip(4, :), z_foot_hip(4, :))


%% Inverse Kinematics

for leg = 1:4
    for time = 1:24
        angle_0 = atan2(z_foot_hip(leg, time), x_foot_hip(leg, time));
        angle_1 = acos((L1 * L1 + x_foot_hip(leg, time) * x_foot_hip(leg, time) + z_foot_hip(leg, time) * z_foot_hip(leg, time) - L2 * L2)/(2 * L1 * sqrt(x_foot_hip(leg, time) * x_foot_hip(leg, time) + z_foot_hip(leg, time) * z_foot_hip(leg, time))));
        theta_1(leg, time) = (angle_0 - angle_1)
        theta_2(leg, time) = pi - acos((L1 * L1 + L2 * L2 - x_foot_hip(leg, time) * x_foot_hip(leg, time) - z_foot_hip(leg, time)*z_foot_hip(leg, time))/(2 * L1 * L2))
    end
end

%Supporting Phase values
% for leg = 1:4
%     for time = 7:24
%         angle_0 = atan2(z_foot_hip(leg, time), x_foot_hip(leg, time));
%         angle_1 = acos((L1 * L1 + x_foot_hip(leg, time) * x_foot_hip(leg, time) + z_foot_hip(leg, time) * z_foot_hip(leg, time) - L2 * L2)/(2 * L1 * sqrt(x_foot_hip(leg, time) * x_foot_hip(leg, time) + z_foot_hip(leg, time) * z_foot_hip(leg, time))));
%         theta_1(leg, time) = (angle_0 - angle_1);
%         theta_2(leg, time) = pi - acos((L1 * L1 + L2 * L2 - x_foot_hip(leg, time) * x_foot_hip(leg, time) - z_foot_hip(leg, time)*z_foot_hip(leg, time))/(2 * L1 * L2));
%     end
% end

figure;
% comment v v v
plot(T_pack, theta_1(1, :))
% xlabel('Time (sec)')
% ylabel('Theta 1 (rad)')
% comment^ ^ ^
% figure;

hold on
plot(T_pack, theta_2(1, :))
xlabel('Time')
ylabel('Joint Position (rad)')
legend('Theta 1','Theta 2')
hold off

%% Plan Velocity Foot Trajectory
y_dot_foot_hip = 0;

for time = 1:24
    x_dot_foot_hip(time) = x_dot_foot(time) - x_dot_hip;
    z_dot_foot_hip(time) = z_dot_foot(time) - z_dot_hip;
end

x_dot_foot_hip;
z_dot_foot_hip;

for row = 1:6
    for col = 1:24
        x_dotf_H(row, col) = [cos(lim_H(row)), -sin(lim_H(row)), 0] * [x_dot_foot_hip(col); y_dot_foot_hip; z_dot_foot_hip(col)];
        y_dotf_H(row, col) = [sin(lim_H(row)), cos(lim_H(row)), 0] * [x_dot_foot_hip(col); y_dot_foot_hip; z_dot_foot_hip(col)];
        z_dotf_H(row, col) = z_dot_foot_hip(col);
%        stopper
    end
end


for time = 1:24
    for leg = 1:4
        T1 = theta_1(leg, time);
        T2 = theta_2(leg, time);
        J_v(1, 1) = -L1 * sin(T1) - L2 * sin(T1 + T2);
        J_v(1, 2) = -L2 * sin(T1 + T2);
        J_v(2, 1) = L1 * cos(T1) + L2 * cos(T1 + T2);
        J_v(2, 2) = L2 * cos(T1 + T2);
        theta_dot(leg, time, :) = inv(J_v) * [x_dot_foot_hip(time); z_dot_foot_hip(time)];
    end
end


theta_dot;


figure;
plot(T_pack, theta_dot(1, :, 1))
hold on
plot(T_pack, theta_dot(1, :, 2))
xlabel('Time (sec)')
ylabel('Angular Velocity (rad/sec)')
legend('Omega 1','Omega 2')

hold off
%Combined walking gait animation

time1 = 6;
time2 = 18;
time3 = 12;
time4 = 0;

figure;
xlim([-2 * L_stride, 2 * L_stride])
ylim([-H_robot, H_robot])
zlim([-H_robot, H_robot])
view(0, 30)

line([0, 0], [0, 0.5], [0, 0])
line([0, -1], [0, 0], [0, 0])
line([-1, -1], [0, 0.5], [0, 0])
line([-1, 0], [0.5, 0.5], [0, 0])
time = 1;
leg_1_thigh = line([0, L1 * cos(theta_1(1, time + time1))], [0.5, 0.5], [0, L1 * sin(theta_1(1, time + time1))]);
leg_1_shin = line([L1 * cos(theta_1(1, time + time1)), x_foot_hip(1, time + time1)], [0.5, 0.5], [L1 * sin(theta_1(1, time + time1)), z_foot_hip(1, time + time1)]);

leg_2_thigh = line([0, L1 * cos(theta_1(2, time + time2))], [0, 0], [0, L1 * sin(theta_1(2, time + time2))]);
leg_2_shin = line([L1 * cos(theta_1(2, time + time2)), x_foot_hip(2, time + time2)], [0, 0], [L1 * sin(theta_1(2, time + time2)), z_foot_hip(2, time + time2)]);

leg_3_thigh = line([-1, (L1 * cos(theta_1(1, time + time3))) - 1],[0.5, 0.5], [0, L1 * sin(theta_1(1, time + time3))]);
leg_3_shin = line([(L1 * cos(theta_1(1, time + time3))) - 1, (x_foot_hip(1, time + time3)) - 1], [0.5, 0.5], [L1 * sin(theta_1(1, time + time2)), z_foot_hip(1, time + time2)]);

leg_4_thigh = line([-1, (L1 * cos(theta_1(4,time + time4))) - 1], [0, 0], [0,L1 * sin(theta_1(4, time + time4))]);
leg_4_shin = line([(L1 * cos(theta_1(4, time + time4))) - 1, (x_foot_hip(4, time + time4)) - 1], [0,0], [L1 * sin(theta_1(4, time + time4)), z_foot_hip(4, time + time4)]);


for cycler = 0:0.01:T
    if cycler == phi(1)*T

        disp("Syncronize legs")

        delete(leg_4_thigh)
        delete(leg_4_shin)

        delete(leg_3_thigh)
        delete(leg_3_shin)

        delete(leg_2_thigh)
        delete(leg_2_shin)

        delete(leg_1_thigh)
        delete(leg_1_shin)

        for time = 1:24

            if time > 18
                time1 = -18;
            end
            if time > 6
                time2 = -6;
            end
            if time > 12
                time3 = -12;
            end

            leg_1_thigh = line([0, L1 * cos(theta_1(1, time + time1))], [0.5, 0.5], [0, L1 * sin(theta_1(1, time + time1))]);
            leg_1_shin = line([L1 * cos(theta_1(1, time + time1)), x_foot_hip(1, time + time1)], [0.5, 0.5], [L1 * sin(theta_1(1, time + time1)), z_foot_hip(1, time + time1)]);

            leg_2_thigh = line([0, L1 * cos(theta_1(2, time + time2))], [0, 0], [0, L1 * sin(theta_1(2, time + time2))]);
            leg_2_shin = line([L1 * cos(theta_1(2, time + time2)), x_foot_hip(2, time + time2)], [0, 0], [L1 * sin(theta_1(2, time + time2)), z_foot_hip(2, time + time2)]);

            leg_3_thigh = line([-1, (L1 * cos(theta_1(1, time + time3))) - 1],[0.5, 0.5], [0, L1 * sin(theta_1(1, time + time3))]);
            leg_3_shin = line([(L1 * cos(theta_1(1, time + time3))) - 1, (x_foot_hip(1, time + time3)) - 1], [0.5, 0.5], [L1 * sin(theta_1(1, time + time3)), z_foot_hip(1, time + time3)]);

            leg_4_thigh = line([-1, (L1 * cos(theta_1(4,time + time4))) - 1], [0, 0], [0,L1 * sin(theta_1(4, time + time4))]);
            leg_4_shin = line([(L1 * cos(theta_1(4, time + time4))) - 1, (x_foot_hip(4, time + time4)) - 1], [0,0], [L1 * sin(theta_1(4, time + time4)), z_foot_hip(4, time + time4)]);
            pause(0.5)
            if time == 24
                continue
            end

            delete(leg_4_thigh)
            delete(leg_4_shin)

            delete(leg_3_thigh)
            delete(leg_3_shin)

            delete(leg_2_thigh)
            delete(leg_2_shin)

            delete(leg_1_thigh)
            delete(leg_1_shin)
        end
    end
end