function [ tr, tt ] = call_icp3( m, d )
%ICP1 call the ICP1 implementation
%   Standardize the inputs/outputs 

    % Add library to path
    if ~exist('icp3.m', 'file')  
        addpath('./icp3/'); 
    end
    
    % Do it!
    [ tt, q, ~ ] = icp3(m,d);
    
    % Convert from quaterion to rotation matrix
    [az, ay, ax] = quat2angle(q');
    tr = rotx(-ax) * roty(-ay) * rotz(-az);
    
    tt = -tt;
end

