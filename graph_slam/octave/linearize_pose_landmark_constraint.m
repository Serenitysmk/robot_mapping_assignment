% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  theta = x(3);
  tx = [x(1) ; x(2)];
  Rx = [cos(theta) , sin(theta);
        -sin(theta) , cos(theta)];
  expected_measure = Rx * (l - tx);
  e = expected_measure - z;
  A = [-cos(theta) , -sin(theta) , -sin(theta) * (l(1) - x(1)) + cos(theta) * (l(2) - x(2)) ;
       sin(theta) , -cos(theta) , -cos(theta) * (l(1) - x(1)) - sin(theta) * ( l(2) - x(2) )];
       
  B = Rx;

end;
