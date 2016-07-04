function [ Omega, Xi ] = Cal_InfoMatVec(p1, p2, I, e12 )
%CAL_INFOMATVEC_V2 Summary of this function goes here
%   Detailed explanation goes here
dim_1 = numel(p1);
dim_2 = numel(p2);
sz_I = size(I);
dim_e = numel(e12);

if sz_I ~= [dim_2, dim_2]
    error('Dimension error!!!');
end
if dim_2 ~= dim_e
    error('Dimension error!!!');
end

dim_out = dim_1+dim_2;
Omega = zeros(dim_out,dim_out);
Xi = zeros(dim_out,1);

switch dim_2
    case 3
        x1 = p1(1); y1 = p1(2); theta1 = p1(3);
        x2 = p2(1); y2 = p2(2); theta2 = p2(3);
        e12(3) = Trans_Period(e12(3), theta2-theta1+pi, theta2-theta1-pi);
        J = [ -cos(theta1), -sin(theta1), sin(theta1)*(x1 - x2) - cos(theta1)*(y1 - y2),  cos(theta1), sin(theta1), 0;
            sin(theta1), -cos(theta1), cos(theta1)*(x1 - x2) + sin(theta1)*(y1 - y2), -sin(theta1), cos(theta1), 0;
            0, 0, -1, 0, 0, 1];
        R1 = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
        T = [R1.', [0;0]; 0 0 1];
        h12 = T*(-p1+p2);
        
        % e12_bar: intermediate vector
        e12_bar = e12 - h12 + J*[p1;p2];
        
        Omega = J'*I*J;
        Xi = J'*I*e12_bar;
        
    case 2        
        x1 = p1(1); y1 = p1(2); theta1 = p1(3);
        x2 = p2(1); y2 = p2(2);
        J = [ -cos(theta1), -sin(theta1), sin(theta1)*(x1 - x2) - cos(theta1)*(y1 - y2),  cos(theta1), sin(theta1);
            sin(theta1), -cos(theta1), cos(theta1)*(x1 - x2) + sin(theta1)*(y1 - y2), -sin(theta1), cos(theta1)];
        R1 = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
        h12 = R1.'*[ x2-x1; y2-y1];
        e12_bar = e12 - h12 + J*[p1;p2];
        Omega = J'*I*J;
        Xi = J'*I*e12_bar;
        
    otherwise
        error('Dimension error!!!');        
end

end