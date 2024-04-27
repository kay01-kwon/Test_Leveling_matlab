clear all
clc
close all

% Robot parameter information
q_pan_init_deg = [0;-120;120];
q_pan_goal_deg = [30;20;10];
q_lift_deg = [45;45;45];

q_pan_rad = pi/180*(q_pan_init_deg + q_pan_goal_deg);
q_lift_rad = pi/180*q_lift_deg;

t_pan_joint =   [200 200*cosd(120) 200*cosd(-120);
                 0 200*sind(120) 200*sind(-120);
                 0 0 0];

t_center_height =   [0;0;-170];

LIFT_p = [0 0 0;
    0 0 0;
    -400 -400 -400;
    1 1 1];


% Obstacle height
h = 100;

% Coordinate transformation (Motor) from wheel center frame to Body one
for i = 1:3
    PAN_p_leg(:,i) = SE3_transform(Rotm_LIFT(q_lift_rad(i)),zeros(3,1))*LIFT_p(:,i);
    B_p_leg(:,i) = SE3_transform(Rotm_PAN(q_pan_rad(i)),t_pan_joint(:,i))*PAN_p_leg(:,i);
    B_p_wc(:,i) = SE3_transform(eye(3), t_center_height)*B_p_leg(:,i);
end

B_p_leg

B_p_wc


Dx12 = B_p_wc(1,1) - B_p_wc(1,2);
Dy12 = B_p_wc(2,1) - B_p_wc(2,2);
Dz12 = B_p_wc(3,1) - B_p_wc(3,2);

Dx13 = B_p_wc(1,1) - B_p_wc(1,3);
Dy13 = B_p_wc(2,1) - B_p_wc(2,3);
Dz13 = B_p_wc(3,1) - B_p_wc(3,3);

A = [-Dx12, Dy12, Dz12;
    -Dx13, Dy13, Dz13];

a_sol = inv(A(1:2,1:2))*[h;h];

% Compute the amount of pitch caused by the obstacle
pitch = asin(a_sol(1));
roll = asin(a_sol(2)/cos(pitch));


Rotm = Rotm_BODY(pitch,roll);

% Coordinate transformation from body frame to world one
for i = 1:3
    W_p_pan_joint(:,i) = SE3_transform(Rotm,zeros(3,1))*[t_pan_joint(:,i);1];
    W_p_leg(:,i) = SE3_transform(Rotm,zeros(3,1))*B_p_leg(:,i);
    W_p_wc(:,i) = SE3_transform(Rotm,zeros(3,1))*B_p_wc(:,i);
end

p = SE3_transform(Rotm,zeros(3,1))*(B_p_wc(:,1)-B_p_wc(:,2));
h_est = p(3)

% Corrected lift
q_lift1_cor = acos((-B_p_wc(3,2)-h_est-170)/400);
q_lift_cor = q_lift_rad;
q_lift_cor(1) = q_lift1_cor;


for i = 1:3
    PAN_p_leg_cor(:,i) = SE3_transform(Rotm_LIFT(q_lift_cor(i)),zeros(3,1))*LIFT_p(:,i);
    B_p_leg_cor(:,i) = SE3_transform(Rotm_PAN(q_pan_rad(i)),t_pan_joint(:,i))*PAN_p_leg_cor(:,i);
    B_p_wc_cor(:,i) = SE3_transform(eye(3), t_center_height)*B_p_leg_cor(:,i);

    W_p_pan_joint_cor(:,i) = SE3_transform(eye(3),zeros(3,1))*[t_pan_joint(:,i);1];
    W_p_leg_cor(:,i) = SE3_transform(eye(3),zeros(3,1))*B_p_leg_cor(:,i);
    W_p_wc_cor(:,i) = SE3_transform(eye(3),zeros(3,1))*B_p_wc_cor(:,i);
end 

figure(1)
% Plot robot represented in the body frame
draw_robot(t_pan_joint,B_p_leg,B_p_wc)

figure(2)
draw_robot(W_p_pan_joint,W_p_leg,W_p_wc)

figure(3)
draw_robot(W_p_pan_joint_cor,W_p_leg_cor,W_p_wc_cor)


