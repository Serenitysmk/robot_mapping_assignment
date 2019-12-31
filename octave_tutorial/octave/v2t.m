%this function takes vector as input
%transform the vector form of the robot pose into a homogenous matrix form
function [t] = v2t(v)
  t = zeros(3,3);
  t(1,1) = cos(v(3));
  t(1,2) = -sin(v(3));
  t(2,1) = sin(v(3));
  t(2,2) = cos(v(3));
  t(1,3) = v(1);
  t(2,3) = v(2);
  t(3,3) = 1.0;
endfunction
