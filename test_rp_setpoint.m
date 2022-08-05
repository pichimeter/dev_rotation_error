% clc, clear all
%%


N = 20;
roll_stick  = (-N:1:N).' / N;
pitch_stick = (-N:1:N).' / N;

ij_color_pos = N+1 + N;
ij_color_neg = N+1 - N;

userAngleLimit = 80;

% rpy = [atan2( CEB(3,2), CEB(3,3)), ...
%        atan2(-CEB(3,1), sqrt(CEB(3,2)^2 + CEB(3,3)^2)), ...
%        atan2( CEB(2,1), CEB(1,1))];
getRollFromVector  = @(v) atan2( v(2), v(3));
getPitchFromVector = @(v) pi/2 - acos(-v(1)); %atan2(-v(1), sqrt(v(2)^2 + v(3)^2));

EeBz = [];
BeEz = [];
try_0 = [];
EeBz_ij = [];
BeEz_ij = [];
try_0_ij = [];
for i = 1:2*N + 1
    for j = 1:2*N + 1
        
        % this would be an alternative solution
        ang  = [-pitch_stick(i), -roll_stick(j), 0] * userAngleLimit * pi/180;
        RxRy = quat2CEB(rpy2quat(ang));
        EeBz(end+1,:) = RxRy(:,3).';
        
        % this is how bf actually maps the sticks
        ang  = [roll_stick(i), pitch_stick(j), 0] * userAngleLimit * pi/180;
        RyRx = quat2CEB(rpy2quat(ang));
        BeEz(end+1,:) = RyRx(3,:);
        
        % this is the mapping it to a half sphere thing
        y = -sin(roll_stick(i)  * userAngleLimit * pi/180);
        x =  sin(pitch_stick(j) * userAngleLimit * pi/180);
        x_abs = abs(x);
        y_abs = abs(y);
        l = 1.0;
        if x_abs > y_abs && x_abs > 0
            l = y_abs / x_abs;
        elseif y_abs > x_abs && y_abs > 0
            l = x_abs / y_abs;           
        end
        s = sqrt(l^2 + 1);
        x = -x / s;
        y = -y / s;
        z = 0;
        if x^2 + y^2 < 1
            z = sqrt(1 - (x^2 + y^2));
        end
        try_0(end+1,:) = [x, y, z];

        if i == ij_color_pos || j == ij_color_pos || ...
           i == ij_color_neg || j == ij_color_neg
            EeBz_ij(end+1,:)  = EeBz(end,:);
            BeEz_ij(end+1,:)  = BeEz(end,:);
            try_0_ij(end+1,:) = try_0(end,:);
        end
        
        if i == 10 && j == 15
            roll_stick(i) * userAngleLimit  * pi/180 * 180/pi
            pitch_stick(j) * userAngleLimit * pi/180 * 180/pi
            getRollFromVector (EeBz(end,:)) * 180/pi
            getPitchFromVector(EeBz(end,:)) * 180/pi
            getRollFromVector (BeEz(end,:)) * 180/pi
            getPitchFromVector(BeEz(end,:)) * 180/pi
            getRollFromVector (try_0(end,:)) * 180/pi
            getPitchFromVector(try_0(end,:)) * 180/pi
            v = try_0(end,:);
        end
        
    end
end

LineWidth = 3;

figure(1)
plot3(EeBz(:,1), EeBz(:,2), EeBz(:,3), 'b.'), grid on, hold on
plot3(EeBz_ij(:,1), EeBz_ij(:,2), EeBz_ij(:,3), 'k.')
Rs = eye(3);
quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off'), hold off
axis equal, axis([-1 1 -1 1 0 1]),view(65, 23)
xlabel('x'), ylabel('y'), zlabel('z'), title('Ry^T(phi)*Rx^T(roll)')

figure(2)
plot3(BeEz(:,1), BeEz(:,2), BeEz(:,3), '.', 'color', [0 0.5 0]), grid on, hold on
plot3(BeEz_ij(:,1), BeEz_ij(:,2), BeEz_ij(:,3), 'k.')
Rs = eye(3);
quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off'), hold off
axis equal, axis([-1 1 -1 1 0 1]),view(65, 23)
xlabel('x'), ylabel('y'), zlabel('z'), title('Ry(theta)*Rx(phi)')

figure(3)
plot3(try_0(:,1), try_0(:,2), try_0(:,3), 'r.'), grid on, hold on
plot3(try_0_ij(:,1), try_0_ij(:,2), try_0_ij(:,3), 'k.')
Rs = eye(3);
quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off'), hold off
axis equal, axis([-1 1 -1 1 0 1]),view(65, 23)
xlabel('x'), ylabel('y'), zlabel('z'), title('Half Sphere Map')

%%

% CEB(t) =
% [ cos(theta(t)), sin(phi(t))*sin(theta(t)), cos(phi(t))*sin(theta(t))]
% [             0,               cos(phi(t)),              -sin(phi(t))]
% [-sin(theta(t)), cos(theta(t))*sin(phi(t)), cos(phi(t))*cos(theta(t))]

% [ cos(-phi), sin(-theta)*sin(-phi), cos(-theta)*sin(-phi)]
% [         0,           cos(-theta),          -sin(-theta)]
% [-sin(-phi), cos(-phi)*sin(-theta), cos(-theta)*cos(-phi)]

% [ cos(-roll), sin(-pitch)*sin(-roll), cos(-pitch)*sin(-roll)]
% [         0,           cos(-pitch),          -sin(-pitch)]
% [-sin(-roll), cos(-roll)*sin(-pitch), cos(-pitch)*cos(-roll)]

% [ cos(roll),  sin(pitch)*sin(roll ), -cos(pitch)*sin(roll )]
% [         0,             cos(pitch),             sin(pitch)]
% [ sin(roll), -cos(roll )*sin(pitch),  cos(pitch)*cos(roll )]





















