% reference trajectory generator (right angle)
function sum_reference_p = rightAngleGenerator(start_p, point_distance, point_num) 
reference_v = point_distance;
reference_w = 0;
reference_p = start_p;
for i = 1:1:point_num
    if i == point_num / 2
        reference_p(3,1) = reference_p(3,1) + pi / 2;
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