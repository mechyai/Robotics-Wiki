function [T01] = transformationMatrixFromDhTable(distance_d_i, angle_theta_i, distance_a_i_minus1, angle_alpha_i_minus1)
% Given the 4 parameters from a row in Denavit-Hartenburg table this function will
% output the transformation matrix of frame {i} referenced from frame {i-1}
%   input angles (alpha and theta) should be in degree form [cosd() and
%   sind() are used]
%   j = i -1
ti = angle_theta_i;
alj = angle_alpha_i_minus1;
di = distance_d_i;
aj = distance_a_i_minus1;

c_ti = cosd(ti);
s_ti = sind(ti);
c_alj = cosd(alj);
s_alj = sind(alj);

% T01 = [c_ti, -s_ti*c_ali, s_ti*s_ali, aj*c_ti;
%     s_ti, c_ti*c_ali, -c_ti*s_ali, aj*s_ti;
%     0, s_ali, c_ali, di;
%     0, 0, 0, 1];
T01 = [c_ti, -s_ti, 0, aj;
    s_ti*c_alj, c_ti*c_alj, -s_alj, s_alj*di;
    s_ti*s_alj, c_ti*s_alj, c_alj, c_alj*di;
    0, 0, 0, 1];

end

