function R = roty(t)
%ROTX Rotation matrix around y axis
%   INPUT: Theta
%   OUTPUT: Rotation Matrix
    ct = cos(t);
    st = sin(t);
    R = [ ct  0   st
          0   1   0
         -st  0   ct ];
end

