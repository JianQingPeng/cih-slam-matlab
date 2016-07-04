function [ output_args ] = Trans_Mat_Pose( input_args )
%TRANSMATPOSE Summary of this function goes here
%   Detailed explanation goes here

sz = size(input_args);
if isequal(sz, [3 1])
    TransType = 'PoseMat2D1';
elseif isequal(sz, [1 3])
    TransType = 'PoseMat2D2';
elseif isequal(sz, [3 3])
    TransType = 'MatPose2D';
end

switch TransType
    case 'PoseMat2D1'
        pose = input_args;
        theta = pose(3);
        x = pose(1);
        y = pose(2);
        T = [cos(theta) -sin(theta) x;...
            sin(theta) cos(theta) y;...
            0 0 1];
        output_args = T;
    case 'PoseMat2D2'
        pose = input_args.';
        theta = pose(3);
        x = pose(1);
        y = pose(2);
        T = [cos(theta) -sin(theta) x;...
            sin(theta) cos(theta) y;...
            0 0 1];
        output_args = T;
    case 'MatPose2D'
        T = input_args;
        pose = zeros(3,1);
        pose(1) = T(1,3);
        pose(2) = T(2,3);
        pose(3) = atan2(T(2,1),T(1,1));
        output_args = pose;
end
        
        
        
end

