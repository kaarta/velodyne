clc;
clear all;
close all;

laser_id = 0 : 31;

vert_correction = [-25 -1 -1.667 -15.639 -11.31 0 -0.667 -8.843 -7.254 0.333 -0.333 -6.148 -5.333 1.333 0.667 ...
                   -4 -4.667 1.667 1 -3.667 -3.333 3.333 2.333 -2.667 -3 7 4.667 -2.333 -2 15 10.333 -1.333];

rot_correction = [1.4 -4.2 1.4 -1.4 1.4 -1.4 4.2 -1.4 1.4 -4.2 1.4 -1.4 4.2 -1.4 4.2 ...
                 -1.4 1.4 -4.2 1.4 -4.2 4.2 -1.4 1.4 -1.4 1.4 -1.4 1.4 -4.2 4.2 -1.4 1.4 -1.4];

[vert_correction_sorted, ind] = sort(vert_correction); 
[ind_sort, ind2] = sort(ind);
             
config = [laser_id' ind2' - 1 vert_correction' vert_correction' * pi / 180 rot_correction' rot_correction' * pi / 180];

fprintf('%d	%d	%.3f	%.8f	%.3f	%.8f\n', config');
fprintf('\n\n');
fprintf('%d	%.3f\n', [laser_id; vert_correction_sorted]);
fprintf('\n\n');