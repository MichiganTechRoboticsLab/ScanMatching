function R = rotx(t)
%ROTX Rotation matrix around x axis
%   INPUT: Theta
%   OUTPUT: Rotation Matrix
    ct = cos(t);
    st = sin(t);
    R = [ 1   0    0
          0   ct  -st
          0   st   ct ];
end

