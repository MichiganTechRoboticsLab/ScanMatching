function [ret] = killoutliers(p, q, error)
% KILLOUTLIERS - removes all points from q that do not have a matching
% point in q within radius defined by error.
%
% This function look at all the points in q and sees if there are no close
% points if p.  if so, it will get rid of the point.

    ret = [];

    parfor ii=1:size(q,2);

        x = q(1,ii);
        y = q(2,ii);

        [d,~] = min((p(1,:)-x).^2 + (p(2,:)-y).^2);
        d = sqrt(d);
        if d < error
            ret = [ret real([x;y])];
        end
    end

end