% From study 2
% m = 0.8;
% Ix = 5.17e-3;
% Iy = 5.17e-3;
% Iz = 1.7e-2;
% Kt = 2e-4;
% Kd = 7e-5;
% l = 0.3;

% From study 1
m = 1;
Ix = 0.11;
Iy = 0.11;
Iz = 0.04;
Kt = 3e-6;
Kd = 4e-9;
l = 0.2;
g = 9.81;

time = 5;
ts = 0.02;
N = time/ts;

goal = [1 1 1 0]';

% X, X', Y, Y', Z, Z', roll, roll', pitch, pitch', yaw, yaw'
A = [
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 g 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 g 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0
];

% F, Tx, Ty, Tz
% F = F1 + F2 + F3 + F4
% Fi = Kt * wi^2
B = [
    0   0   0   0;
    0   0   0   0;
    0   0   0   0;
    0   0   0   0;
    0   0   0   0;
    1/m 0   0   0;
    0   0   0   0;
    0 1/Ix  0   0;
    0   0   0   0;
    0   0  1/Iy 0;
    0   0   0   0;
    0   0   0 1/Iz
    ];

% C = [
%     1 0 0 0 0 0 0 0 0 0 0 0;
%     0 0 1 0 0 0 0 0 0 0 0 0;
%     0 0 0 0 1 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 1 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 1 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 0
%     ];
C = [
    1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0
    ];
D = [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0
    ];

sys = c2d(ss(A,B,C,D), ts);
% y = [X, Y, Z, roll, pitch, yaw]

% Q = diag([21, 6, 14, 6, 81, 6, 11, 9, 16, 6, 56000, 6]);
c = 0.001;
% Q = diag([c, c, c, c, c, c, c, c, c, c, c, c]);
Q = C'*C;
Q_int = [C'*C zeros([12,4])
    zeros([4,12]) eye(4)];
% Q = 0.005*Q;
%Q(5,5) = 0.3;
%Q(6,6) = 0.08;
% Q(5,5) = 10;
% Q(7,7) = 10;
% Q(9,9) = 10;
% R = diag([0.1 0.1 0.1 0.1]);
R = diag([1 1 1 1]);
[K s e] = lqr(sys, Q, R);
[K_int s_int e_int] = lqi(sys, Q_int, R);
% K = K*1e14;
% u = -K*xv + r

% X, X', Y, Y', Z, Z', roll, roll', pitch, pitch', yaw, yaw'
x = [0 0 0 0 0 0 pi/12 0 0 0 0 0]';
last_err = [0 0 0];

[w1, w2, w3, w4] = solve_drone_input(Kt, Kd, l);

[t, y] = ode113(@(t, y) sys_int(t, y, A, B, K, K_int, Kt, m, l, Ix, Iy, Iz, g, w1, w2, w3, w4, goal), 0:ts:time, x);

figure(1);
plot(t, y(:, 1), t, y(:, 3), t, y(:, 5));
title('X, Y, Z');
xlabel('Time (s)');
ylabel('Dist (m)');
legend({'X', 'Y', 'Z'});

figure(2);
plot(t, y(:, 7), t, y(:, 9), t, y(:, 11));
title('Roll, Pitch, Yaw');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend({'Roll', 'Pitch', 'Yaw'});

figure(3);
plot(t, y(:, 2), t, y(:, 4), t, y(:, 6));
title('Xd, Yd, Zd Speeds');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend({'Xd', 'Yd', 'Zd'});

figure(4);
plot(t, y(:, 2), t, y(:, 4), t, y(:, 6));
title('Rolld, Pitchd, Yawd Speeds');
xlabel('Time (s)');
ylabel('Angular Speed (rad/s)');
legend({'Rolld', 'Pitchd', 'Yawd'});

function [w1_out, w2_out, w3_out, w4_out] = sub_ws(w1, w2, w3, w4, F_out, Tx_out, Ty_out, Tz_out)
    % w1_out = double(subs(w1, [F, Tx, Tz], [F_out, Tx_out, Tz_out]));
    w1_out = w1(F_out, Tx_out, Tz_out);
    % w2_out = double(subs(w2, [F, Ty, Tz], [F_out, Ty_out, Tz_out]));
    w2_out = w2(F_out, Ty_out, Tz_out);
    % w3_out = double(subs(w3, [F, Tx, Tz], [F_out, Tx_out, Tz_out]));
    w3_out = w3(F_out, Tx_out, Tz_out);
    % w4_out = double(subs(w4, [F, Ty, Tz], [F_out, Ty_out, Tz_out]));
    w4_out = w4(F_out, Ty_out, Tz_out);
end


function dx = sys_int(t, state, A, B, K, K_int, Kt, m, l, Ix, Iy, Iz, g, w1, w2, w3, w4, goal)

    % u = -K*state;
    u = -K_int*[state;
        goal];
        
    F_out = u(1);
    Tx_out = u(2);
    Ty_out = u(3);
    Tz_out = u(4);
    
    [F1, F2, F3, F4] = sub_ws(w1, w2, w3, w4, F_out, Tx_out, Ty_out, Tz_out);
    
    F1 = F1 + m*g/4;
    F2 = F2 + m*g/4;
    F3 = F3 + m*g/4;
    F4 = F4 + m*g/4;
    
    % phi = roll, theta = pitch, gamma = yaw
    phi = state(7);
    phid = state(8);
    theta = state(9);
    thetad = state(10);
    gamma = state(11);
    gammad = state(12);
    Xdd = ((F1+F2+F3+F4)*(cos(phi)*sin(theta)*cos(gamma) + sin(phi)*sin(gamma)))/m;
    Ydd = ((F1+F2+F3+F4)*(cos(phi)*sin(theta)*sin(gamma) + sin(phi)*cos(gamma)))/m;
    Zdd = ((F1+F2+F3+F4)*(cos(phi)*cos(theta)) - m*g)/m;
    phidd = ((F1-F3)*l + thetad*gammad*(Iy - Iz))/Ix;
    thetadd = ((F2 - F4)*l + gammad*phid*(Iz - Ix))/Iy;
    gammadd = ((F2 + F4 - F1 - F3) + phid*thetad*(Ix - Iy))/Iz;
    
    % state X, X', Y, Y', Z, Z', roll, roll', pitch, pitch', yaw, yaw'
    dx = double([state(2) Xdd state(4) Ydd state(6) Zdd state(8) phidd state(10) thetadd state(12) gammadd]');
end