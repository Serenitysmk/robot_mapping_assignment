% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 
 
  % TODO: initialize H and b of the linear system
  H = zeros(9,9);
  b = zeros(1,9);
  
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  m = size(Z,1);
  for i = 1 : m
    e = error_function(i,X,Z);
    J = jacobian(i,Z);
    b = b + e' * J;
    H = H + J' * J;
  endfor

  % TODO: solve and update the solution
  L = chol(H);
  y = -b / L;
  x = y / L';
  X = reshape(x , 3,3);
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  Z_s = Z(i , 1:3);
  Z_m = Z(i , 4:6);
  e = Z_s' - X * Z_m';
  
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  Z_m = Z(i,4:6);
  x = Z_m(1);
  y = Z_m(2);
  theta = Z_m(3);
  J = zeros(3 , 9);
  J(1, 1:3) = [x , y , theta];
  J(2, 4:6) = [x , y , theta];
  J(3, 7:9) = [x , y , theta];
end
