function phi = stanleyController(p_c, sum_reference_p, v_c)
% set controller gains
k = 0.25;

d = size(sum_reference_p);
square_cross_track_error = 10000;
close_path_point_id = 0;
% searching the closest point on the path
for i = 1:1:d(1,2)
    diff_distance = (sum_reference_p(1,i) - p_c(1,1))^2 + (sum_reference_p(2,i) - p_c(2,1))^2;
    if square_cross_track_error > diff_distance
        square_cross_track_error = diff_distance;
        close_path_point_id = i;
    end
end
% calculate cross track error
current_theta = p_c(3,1);
R_transform = [-sin(current_theta) cos(current_theta) 0];
p_y_e = R_transform * (sum_reference_p(:,close_path_point_id) - p_c);
if p_y_e >= 0
    cross_track_error = sqrt(square_cross_track_error);
else 
    cross_track_error = -sqrt(square_cross_track_error);
end
% calcuate path heading angle
if close_path_point_id == d(1,2)
    path_theta = 0;
else
    diff_p = sum_reference_p(:,close_path_point_id + 1) - sum_reference_p(:,close_path_point_id);
    path_theta = atan2(diff_p(2,1), diff_p(1,1));
end
theta_e = path_theta - p_c(3,1);
phi = theta_e + atan(k * (cross_track_error) / v_c);
end


