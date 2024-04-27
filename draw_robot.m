function  draw_robot(t_pan_joint, B_p_leg, B_p_wc)
%DRAW_ROBOT 이 함수의 요약 설명 위치
%   자세한 설명 위치

D = 169;

theta = 0:0.1:2*pi+0.5;

N = length(theta);

for i = 1:N
    for j = 1:3
        x_wheel(i,j) = D/2*cos(theta(i)) + B_p_wc(1,j);
        y_wheel(i,j) = B_p_wc(2,j);
        z_wheel(i,j) = D/2*sin(theta(i)) + B_p_wc(3,j);
    end

hold on
% Plot Base plane
plot3(t_pan_joint(1,1:2), t_pan_joint(2,1:2), t_pan_joint(3,1:2),'color','k')
plot3(t_pan_joint(1,2:3), t_pan_joint(2,2:3), t_pan_joint(3,2:3),'color','k')
plot3([t_pan_joint(1,1), t_pan_joint(1,3)], ...
    [t_pan_joint(2,1), t_pan_joint(2,3)], ...
    [t_pan_joint(3,1), t_pan_joint(3,3)],'color','k')

    for i = 1:3
        % Plot leg
        plot3([t_pan_joint(1,i) B_p_leg(1,i)], ...
            [t_pan_joint(2,i) B_p_leg(2,i)], ...
            [t_pan_joint(3,i) B_p_leg(3,i)], ...
            'color','b', ...
            'LineWidth', 4);
        % Plot leg to wheel center
        plot3([B_p_wc(1,i) B_p_leg(1,i)], ...
            [B_p_wc(2,i) B_p_leg(2,i)], ...
            [B_p_wc(3,i) B_p_leg(3,i)], ...
            'color','b', ...
            'LineWidth',4);
        plot3(x_wheel(:,i),y_wheel(:,i),z_wheel(:,i), ...
            'color','r');
    end

v = [0 -1 0];
view(v);
rotate3d on;

xlabel('x')
ylabel('y')
zlabel('z')

end