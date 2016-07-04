function [ I_res, e_res ] = Fun_Merge_InfoMatVec( I1, e1, I2, e2 )
%FUN_MERGE_INFOMATVEC Summary of this function goes here
%   Detailed explanation goes here

e2(3) = Trans_Period(e2(3), e1(3)+pi, e1(3)-pi);
I_res = I1+I2;
e_res = inv(I_res)*(I1*e1+I2*e2);
e_res(3) = Trans_Period(e_res(3), pi, -pi);

end

