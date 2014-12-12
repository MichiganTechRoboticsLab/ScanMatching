%This function look at all the points in q and sees if there are no close
%points if p.  if so, it will get rid of the point.

function [ret] = killoutliers(p, q, error)

maxdist = 0;
ret = [];

for ii=1:size(q,2);

    x = q(1,ii);
    y = q(2,ii);
    
    [d,~] = min((p(1,:)-x).^2 + (p(2,:)-y).^2);
    d = sqrt(d);
    if d < error
        ret = [ret real([x;y])];
    end
end
figure(2)
plot(q(1,:), q(2,:), 'r.');
hold on
plot(ret(1,:), ret(2,:), 'g.');
hold off

end