function [ tr, tt ] = call_libicp( m, d )
%ICP1 call the ICP1 implementation
%   Standardize the inputs/outputs 

    % Add library to path
    if ~exist('icpmex.cpp', 'file')  
        addpath('./libicp/matlab/'); 
    end

    Tr_fit = icpMex(m,d,eye(3),-1,'point_to_plane');
    tt = Tr_fit([1 2], 3);
    tr = Tr_fit([1 2], [1 2]);
    
end

