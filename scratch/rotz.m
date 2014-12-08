function R = rotz(t)
%ROTX Rotation matrix around x axis
%   INPUT: Theta
%   OUTPUT: Rotation Matrix
    ct = cos(t);
    st = sin(t);
    R = [ ct  -st  0
          st   ct  0
          0    0   1 ];
end

