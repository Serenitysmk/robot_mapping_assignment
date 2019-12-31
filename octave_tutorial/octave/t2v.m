%this function takes homogenous transformation matrix as input
%transform the robot pose in homogenous matrix into a vector form
function [v] = t2v(t)
  v = zeros(3,1);
  v(3) = acos(t(1,1));
  v(1) = t(1,3);
  v(2) = t(2,3);
  normalize_angle(v(3));
endfunction
