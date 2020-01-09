function p = log_odds_to_prob(l)
% Convert log odds l to the corresponding probability values p.
% l could be a scalar or a matrix.

% TODO: compute p.

p = ones(size(l)) - (ones(size(l)) ./ (ones(size(l)) + exp(l)));


end
