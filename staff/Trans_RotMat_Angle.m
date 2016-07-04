function [ output_args ] = Trans_RotMat_Angle( input_args )
%TRANS_ROTMAT_ANGLE Summary of this function goes here
%   Detailed explanation goes here
sz = size(input_args);
if isequal(sz, [1 1])
    TransType = 'AngleMat';
elseif isequal(sz, [2 2])
    TransType = 'MatAngle';
end

switch TransType
    case 'AngleMat'
        theta = input_args;
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        output_args = R;
    case 'MatAngle'
        R = input_args;
        theta = atan2(R(2,1),R(1,1));
        output_args = theta;
end
end

