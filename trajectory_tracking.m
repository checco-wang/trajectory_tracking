clear all;
clc;

%This is to simulating to simsulate the sliding mode controller to track trajectory
        
rate = 5;
dt = 1 / rate;
sim_time = 20;
sim_step = sim_time / dt;

reference_x = 0;
reference_y = 0;
reference_theta = 0;

reference_v = 1.0;
reference_w = 1.0;

% reference trajectory generator
for i = 1:1:sim_step
    reference_theta = reference_p(1,3);
    R_reference_transform = [cos(reference_theta) 0;
                                         sin(reference_theta) 0;
                                                        0           1];
    reference_q = [reference_v;
                          reference_w];
    reference_p_diff = R_transform * reference_q;
    reference_p = reference_p + reference_p_diff;
    whole_reference_p(:, i) = reference_p;
    whole_reference_q(:, i) = reference_q;
end

start_current_x = 0;
start_current_y = 0;
start_current theta = 0;
current_p = [start_current_x; start_current_y; start_current_theta];

for i = 1:1:sim_step 
    reference_p = whole_reference_p(:, i);
    reference_q = whole_reference_q(:, i);

    R_coordinate_transform = [ cos(current_theta) sin(current_theta) 0;
                                          -sin(current_theta) cos(current_theta) 0;
                                                           0                        0        1];
    p_e = R_coordinate_transform * (reference_p - current_p);
    q = smc(p_e, reference_q);
    
    R_velocity_transform = [cos(current_theta) 0;
                                      sin(current_theta) 0;
                                                  0           1];
    p_diff = R_velocity_transform * q;
    current_p = current_p + p_diff * dt;
    whole_current_p(:, i) = current_p;

end;





