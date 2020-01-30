% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    omega = edge.information;
    z12 = edge.measurement;
    Z = v2t(z12);
    err = t2v(inv(Z) * inv(x1) * x2);
    Fx = Fx + err' * omega * err;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    omega = edge.information;
    zij = edge.measurement;
    Rx = [cos(x(3)) sin(x(3))
          -sin(x(3)) cos(x(3))];
    tx = [x(1) ; x(2)];
    expected_measure = Rx * (l - tx);
    err = expected_measure - edge.measurement;
    Fx = Fx + err' * omega * err;
    
   
  end

end
