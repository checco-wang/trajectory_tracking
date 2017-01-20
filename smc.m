function q = smc(p_e, reference_q, v_r_diff, last_v_w)
velocity_limit = 0.5;
w_acc_limit = pi / 3;
dt = 1 / 5.0;
% save gains for large circle tracking
% k1 = 0.1;       % gain for s1
% k2 = 0.1;       % gain for s2
% smooth_factor1 = 0.01;         % smooth chattering
% smooth_factor2 = 0.01;         % smooth chattering

% save gains for circle which radious is 0.4m
% k1 = 0.08;       % gain for s1
% k2 = 5;       % gain for s2
% smooth_factor1 = 0.01;         % smooth chattering
% smooth_factor2 = 2.0;         % smooth chattering

% save gains for circle which radious is 0.1m
k1 = 0.05;       % gain for s1
k2 = 0.05;       % gain for s2
smooth_factor1 = 0.01;         % smooth chattering
smooth_factor2 = 1.2;         % smooth chattering

x_e = p_e(1,1);
y_e = p_e(2,1);
theta_e = p_e(3,1);
v_r = reference_q(1,1);
w_r = reference_q(2,1);
v_r_diff = 0;

s1 = x_e;
s2 = theta_e + atan(v_r * y_e);

w_c = (w_r + (y_e / (1 + (v_r * y_e) ^ 2)) * v_r_diff + (v_r / (1 + (v_r * y_e) ^ 2))...
        * (v_r * sin(theta_e)) + k2 * (s2 / (abs(s2) + smooth_factor2)))...
        / (1 + (v_r / (1 + (v_r * y_e) ^ 2)) * x_e);
v_c = y_e * w_c + v_r * cos(theta_e) + k1 * (s1 / (abs(s1) + smooth_factor1));
if v_c > velocity_limit
    v_c = velocity_limit;
end
if v_c < 0
    v_c = 0;
end
if (w_c < last_v_w - w_acc_limit * dt)
    w_c = last_v_w - w_acc_limit * dt;
end
if (w_c > last_v_w + w_acc_limit * dt)
    w_c = last_v_w + w_acc_limit * dt;
end

if ((w_c - last_v_w) < -w_acc_limit * dt / 2 || (w_c - last_v_w) > w_acc_limit * dt / 2)
    v_c = 0;
end

q = [v_c; w_c];
