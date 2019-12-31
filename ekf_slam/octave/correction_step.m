function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);

% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = [];

x = mu(1);
y = mu(2);
theta = mu(3);

for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
	% If the landmark is obeserved for the first time:
	if(observedLandmarks(landmarkId)==false)
		% TODO: Initialize its pose in mu based on the measurement and the current robot pose:
    mu(2 * landmarkId + 2) = x + z(i).range * cos(z(i).bearing + theta);
    mu(2 * landmarkId + 3) = y + z(i).range * sin(z(i).bearing + theta); 
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
	endif

	% TODO: Add the landmark measurement to the Z vector
  Z(2 * i - 1) = z(i).range;
  Z(2 * i) = z(i).bearing;
	
	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:
  m_x = mu(2 * landmarkId + 2);
  m_y = mu(2 * landmarkId + 3);
  expect_range = sqrt( (m_x - x) * (m_x - x) + (m_y - y) * (m_y - y) ) ;
  expect_bearing = atan2(m_y - y , m_x - x) - theta;
  expect_bearing = normalize_angle(expect_bearing);
  expectedZ(2 * i - 1) = expect_range;
  expectedZ(2 * i) = expect_bearing;

	% TODO: Compute the Jacobian Hi of the measurement function h for this observation
  Hi = zeros(2 , size(mu , 1));

  delta_x = m_x - x;
  delta_y = m_y - y;
  q = delta_x * delta_x + delta_y * delta_y;
  sqrt_q = sqrt(q);
  Hi(1 , 1) = -sqrt_q * delta_x;
  Hi(1 , 2) = -sqrt_q * delta_y;
  Hi(1 , 3) = 0.0;
  Hi(1 , 2 * landmarkId + 2) = sqrt_q * delta_x;
  Hi(1 , 2 * landmarkId + 3) = sqrt_q * delta_y;
  
  Hi(2 , 1) = delta_y;
  Hi(2 , 2) = -delta_x;
  Hi(2 , 3) = -q;
  Hi(2 , 2 * landmarkId + 2) = -delta_y;
  Hi(2 , 2 * landmarkId + 3) = delta_x;
  Hi = (1 / q) * Hi;
 	
	% Augment H with the new Hi
	H = [H;Hi];	
endfor

% TODO: Construct the sensor noise matrix Q
Q = eye(2 * m , 2 * m);
Q = 0.01 * Q;

% TODO: Compute the Kalman gain 
tmp = H * sigma * H' + Q;
tmp_inv = inv(tmp);
Kt = sigma * H' * tmp_inv;
 

% TODO: Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
diff_z = Z - expectedZ;
diff_z = normalize_all_bearings(diff_z);

% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.
mu = mu + Kt * diff_z;
sigma = (eye(size(sigma)) - Kt * H) * sigma;
mu(3) = normalize_angle(mu(3));

end
