function [ tr, tt ] = call_icp1( m, d )
%ICP1 call the ICP1 implementation
%   Standardize the inputs/outputs 

    % Add library to path
    if ~exist('icp1.m', 'file')  
        addpath('./icp1/'); 
    end

    % Requires 3D points
    d  = [d; zeros(1, size(d,2))];
    m  = [m; zeros(1, size(m,2))];
    
    % Do it!
    [tr, tt] = icp1(m,d,5,'Matching','kDtree');
    
end

