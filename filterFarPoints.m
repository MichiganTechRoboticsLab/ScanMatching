function [ ret ] = filterFarPoints( pts, ctr, d )
%fitlereFarPoints filters out points which are distance d from the given
%center
[theta, rho] = cart2pol(pts(1,:) - ctr(1), pts(2,:) - ctr(2));

temp = [theta;rho];
temp(:,temp(2,:)>d)=[];

[ret_x, ret_y] = pol2cart(temp(1,:), temp(2,:));
ret = [ret_x + ctr(1);ret_y + ctr(2)];
end

