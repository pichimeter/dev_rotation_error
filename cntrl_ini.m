clc, clear all
%%

% notes:
% - 

addpath ../fcn_bib

J = 0.005*[1, 1, 1].';
Dw = 0*5e-3*[1 1 1].';

s = tf('s');

krp = J(1) * 130;
Prp_ = 130 * tf(1, [1 0]) * tf(1,[1/(2*pi*10) 1]) / (krp / J(1)); % open-loop plant u->w rp
Prp = krp / J(1) * tf(1, [1 0]) * tf(1,[1/(2*pi*10) 1]);
Crp = 0.8*(1.5 + 23/s + 0.014*s*tf(1,[1/(2*pi*100) 1])*tf(1,[1/(2*pi*100) 1]));
Trp = feedback(Prp*Crp, 1);
figure(1)
bode(krp / J(1) * Prp_, Prp, Trp), grid on

% ky = J(3) * 2;
% Py_ = 2 * tf(1,[1/(2*pi*10) 1]) / (ky / J(3));  % open-loop plant u->w y
% Py = ky / J(3) * tf(1,[1/(2*pi*10) 1]);
% Cy = 1.2*(0.8 + 48/s);
% Ty = feedback(Py*Cy, 1);
% figure(2)
% bode(ky / J(3) * Py_, Py, Ty), grid on

ky = krp;
Py_ = Prp_;
Py = Prp;
Cy = Crp;
Ty = Trp;

Kp = [1 1 1]*0.9;
Jinv = diag(1./J);
k = [krp krp ky];

rpy_0  = [-5, -135, 0] * pi/180;
quat_0 = rpy2quat(rpy_0);
CEB_0  = quat2CEB(rpy2quat(rpy_0));

rpy_setp  = [15, 45, 0] * pi/180;
quat_setp = rpy2quat(rpy_setp);
CEB_setp  = quat2CEB(rpy2quat(rpy_setp));

Tsim = 0.15;
Tsave = Tsim / 50;

%% simulate first

clc

quat_rpy = simout.signals.values(:,1:4);
quat_err = simout.signals.values(:,5:8);
quat_red = simout.signals.values(:,9:12);

% error angle ez
CEB_rpy = quat2CEB(quat_rpy(end,:));
CEB_err = quat2CEB(quat_err(end,:));
CEB_red = quat2CEB(quat_red(end,:));
ez_ang_rpy = acos(CEB_rpy(:,3).' * CEB_setp(:,3)) * 180/pi
ez_ang_err = acos(CEB_err(:,3).' * CEB_setp(:,3)) * 180/pi
ez_ang_red = acos(CEB_red(:,3).' * CEB_setp(:,3)) * 180/pi

% quaternion error
[ang_rpy, ax_rpy] = quatError2AxAng(quat_setp, quat_rpy(end,:));
ang_rpy * 180/pi
[ang_err, ax_err] = quatError2AxAng(quat_setp, quat_err(end,:));
ang_err * 180/pi
[ang_red, ax_red] = quatError2AxAng(quat_setp, quat_red(end,:));
ang_red * 180/pi

% % rpy error without yaw
% quat_setp_conj = quat_setp; quat_setp_conj(2:4) = -quat_setp_conj(2:4);
% rpy_rpy = quat2rpy( ( quat2QprodL(quat_setp_conj) * removeYawFromQuat(quat_rpy(end,:)).' ).' );
% rpy_rpy * 180/pi
% rpy_err = quat2rpy( ( quat2QprodL(quat_setp_conj) * removeYawFromQuat(quat_err(end,:)).' ).' );
% rpy_err * 180/pi
% rpy_red = quat2rpy( ( quat2QprodL(quat_setp_conj) * removeYawFromQuat(quat_red(end,:)).' ).' );
% rpy_red * 180/pi

% rpy error
quat_setp_conj = quat_setp; quat_setp_conj(2:4) = -quat_setp_conj(2:4);
rpy_rpy = quat2rpy( ( quat2QprodL(quat_setp_conj) * (quat_rpy(end,:)).' ).' );
rpy_rpy * 180/pi
rpy_err = quat2rpy( ( quat2QprodL(quat_setp_conj) * (quat_err(end,:)).' ).' );
rpy_err * 180/pi
rpy_red = quat2rpy( ( quat2QprodL(quat_setp_conj) * (quat_red(end,:)).' ).' );
rpy_red * 180/pi

%%

arrow_length = 1;
DashedLine = '--';
SolidLine  = '-';
Point = '.';
LineWidth = 3;

[x y z] = sphere(111);

N = size(quat_rpy, 1);
points_x_rpy = []; points_y_rpy = []; points_z_rpy = [];
points_x_err = []; points_y_err = []; points_z_err = [];
points_x_red = []; points_y_red = []; points_z_red = [];

for i = 1:N
        
    CEB_rpy = quat2CEB(quat_rpy(i,:));
    CEB_err = quat2CEB(quat_err(i,:));
    CEB_red = quat2CEB(quat_red(i,:));
    
    figure(4)
    subplot(131)
    h = surfl(x, y, z); hold on
    set(h, 'FaceAlpha', 0.1)
    shading interp   
    Rs = CEB_rpy * arrow_length;
    points_x_rpy(end+1,:) = Rs(:,1).';
    points_y_rpy(end+1,:) = Rs(:,2).';
    points_z_rpy(end+1,:) = Rs(:,3).';
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    Rs = CEB_0 * arrow_length;
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    Rs = CEB_setp * arrow_length;
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), DashedLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), DashedLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), DashedLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    plot3(points_x_rpy(:,1), points_x_rpy(:,2), points_x_rpy(:,3), Point, 'color', [1 0 0])
    plot3(points_y_rpy(:,1), points_y_rpy(:,2), points_y_rpy(:,3), Point, 'color', [0 0.5 0])
    plot3(points_z_rpy(:,1), points_z_rpy(:,2), points_z_rpy(:,3), Point, 'color', [0 0 1]), hold off  
    view(65, 23)
    axis equal, title('RP Cntrl')
    set(gcf, 'color', 'w');
    
    subplot(132)
    h = surfl(x, y, z); hold on
    set(h, 'FaceAlpha', 0.1)
    shading interp
    Rs = CEB_err * arrow_length;
    points_x_err(end+1,:) = Rs(:,1).';
    points_y_err(end+1,:) = Rs(:,2).';
    points_z_err(end+1,:) = Rs(:,3).';
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    Rs = CEB_0 * arrow_length;
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    Rs = CEB_setp * arrow_length;
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), DashedLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), DashedLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), DashedLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    plot3(points_x_err(:,1), points_x_err(:,2), points_x_err(:,3), Point, 'color', [1 0 0])
    plot3(points_y_err(:,1), points_y_err(:,2), points_y_err(:,3), Point, 'color', [0 0.5 0])
    plot3(points_z_err(:,1), points_z_err(:,2), points_z_err(:,3), Point, 'color', [0 0 1]), hold off  
    view(65, 23)
    axis equal, title('Quat Error Cntrl')
    set(gcf, 'color', 'w');
    
    subplot(133)
    h = surfl(x, y, z); hold on
    set(h, 'FaceAlpha', 0.1)
    shading interp
    Rs = CEB_red * arrow_length;
    points_x_red(end+1,:) = Rs(:,1).';
    points_y_red(end+1,:) = Rs(:,2).';
    points_z_red(end+1,:) = Rs(:,3).';
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
    Rs = CEB_0 * arrow_length;
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    Rs = CEB_setp * arrow_length;
    quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), DashedLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), DashedLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
    quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), DashedLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
    plot3(points_x_red(:,1), points_x_red(:,2), points_x_red(:,3), Point, 'color', [1 0 0])
    plot3(points_y_red(:,1), points_y_red(:,2), points_y_red(:,3), Point, 'color', [0 0.5 0])
    plot3(points_z_red(:,1), points_z_red(:,2), points_z_red(:,3), Point, 'color', [0 0 1]), hold off  
    view(65, 23)
    axis equal, title('Axis z Error Cntrl')
    set(gcf, 'color', 'w');
    drawnow;

%     frame(i) = getframe(gcf);
%     im = frame2im(frame(i));
%     [imind, cm] = rgb2ind(im, 256);
%     if i == 1
%         imwrite(imind, cm, 'slerp_cntrl_gif.gif', 'gif', 'Loopcount', inf);
%     else
%         imwrite(imind, cm, 'slerp_cntrl_gif.gif', 'gif', 'WriteMode', 'append');
%     end 

end

% writerObj = VideoWriter('slerp_cntrl', 'MPEG-4'); % 30 fps default
% writerObj.Quality = 100;
% open(writerObj);
% for i = 1:N 
%     writeVideo(writerObj, frame(i));
% end
% close(writerObj);
