% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);
  e = t2v(inv(Z) * inv(X1) * X2 );
  
  theta_ij = z(3);
  theta_i = x1(3);
  Rij_Ri = [cos(theta_ij) * cos(theta_i) - sin(theta_ij) * sin(theta_i) , cos(theta_ij) * sin(theta_i) + sin(theta_ij) * cos(theta_i);
            -sin(theta_ij) * cos(theta_i) - cos(theta_ij) * sin(theta_i) , -sin(theta_ij) * sin(theta_i) + cos(theta_ij) * cos(theta_i)];
  xi = x1(1);
  yi = x1(2);
  xj = x2(1);
  yj = x2(2);  
  A = zeros(3,3);
  A(1,1) = -Rij_Ri(1,1);
  A(1,2) = -Rij_Ri(1,2);
  A(2,1) = -Rij_Ri(2,1);
  A(2,2) = -Rij_Ri(2,2);
 
  A(1:2,3) = [cos(theta_ij)*(-sin(theta_i)*(xj-xi)+cos(theta_i)*(yj-yi))+sin(theta_ij)*(-cos(theta_i)*(xj-xi)-sin(theta_i)*(yj-yi)); -sin(theta_ij)*(-sin(theta_i)*(xj-xi)+cos(theta_i)*(yj-yi))+cos(theta_ij)*(-cos(theta_i)*(xj-xi)-sin(theta_i)*(yj-yi))]; 
  A(3,3) = -1;
  
  B = zeros(3,3);
  B(1,1) = Rij_Ri(1,1);
  B(1,2) = Rij_Ri(1,2);
  B(2,1) = Rij_Ri(2,1);
  B(2,2) = Rij_Ri(2,2);

  B(3,3) = 1;
 
 


end;
