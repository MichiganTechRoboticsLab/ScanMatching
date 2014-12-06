function [ tr, tt ] = call_icp3( m, d )
%ICP1 call the ICP1 implementation
%   Standardize the inputs/outputs 

    % Add library to path
    if ~exist('icp3.m', 'file')  
        addpath('./icp3/'); 
    end
    
    % Do it!
    [tr, tt] = icp3(m,d);
    
end

