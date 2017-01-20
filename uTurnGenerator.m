% reference trajectory generator (U-shape)
function sum_reference_p = uTurnGenerator(start_p, radius, point_distance, point_num) 
reference_v = point_distance;
reference_p = start_p;
for i = 1:1:point_num
    if i < point_num / 3
        reference_w = 0;
    elseif i < 2 * point_num / 3
        reference_w = reference_v / radius;
    else
        reference_w = 0;
    end
    reference_theta = reference_p(3,1);
    R_reference_transform = [cos(reference_theta) 0;
                             sin(reference_theta) 0;
                                     0           1];
    reference_q = [reference_v;
                   reference_w];
    reference_p_diff = R_reference_transform * reference_q;
    reference_p = reference_p + reference_p_diff;
    sum_reference_p(:,i) = reference_p;
end
end