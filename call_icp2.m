function [ tr, tt ] = call_icp2( m, d )
%ICP1 call the ICP2 implementation
%   Standardize the inputs/outputs 

    % Add library to path
    if ~exist('icp2.m', 'file')  
        addpath('./icp2/'); 
    end

    [tr, tt] = icp2(m,d);
    
end

