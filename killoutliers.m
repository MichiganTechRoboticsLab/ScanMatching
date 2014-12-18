function [q_prime, rejection] = killoutliers(p, q, error)
% KILLOUTLIERS - removes all points from q that do not have a matching
% point in p within radius defined by error.  It also returns the
% percentage of points which it rejected from q

    q_prime = [];

    %optomization
    error = error ^ 2;
    
    parfor ii=1:size(q,2);
        x = q(1,ii);
        y = q(2,ii);

        [d,~] = min((p(1,:)-x).^2 + (p(2,:)-y).^2);
        if d < error
            q_prime = [q_prime [x;y]];
        end
    end
    
    rejection = 1 - size(q_prime,2) / size(q,2);

end