function [ Omega, Xi ] = Add_InfoMatVec_v2( Omega, Xi, Omega_add, Xi_add, vec_1, vec_2 )
%ADD_INFOMATVEC_V2 Summary of this function goes here
%   Detailed explanation goes here

dim_add = numel(Xi_add);
dim_1 = numel(vec_1);
dim_2 = numel(vec_2);

if 0 ~= dim_add-dim_1-dim_2
    error('Dimension Error In Function Add InfoMatVec!!!');
end

if dim_1 == 3
    vec_1_add = (1:3);
    vec_2_add = (4:dim_add);
elseif dim_1 == 2
    vec_1_add = (1:2);
    vec_2_add = (3:dim_add);
end

% dim = numel(Xi);
% if dim < vec_1(1)
%     db;
% elseif dim < vec_2(1)
%     db;
% end
Omega_add_tmp = Omega_add(vec_2_add,vec_1_add);
Omega(vec_2,vec_1) = Omega(vec_2,vec_1) + Omega_add_tmp;
Omega(vec_2,vec_2) = Omega(vec_2,vec_2) + Omega_add(vec_2_add,vec_2_add);
Omega(vec_1,vec_1) = Omega(vec_1,vec_1) + Omega_add(vec_1_add,vec_1_add);
Omega(vec_1,vec_2) = Omega(vec_1,vec_2) + Omega_add(vec_1_add,vec_2_add);

Xi(vec_1) = Xi(vec_1)+Xi_add(vec_1_add);
Xi(vec_2) = Xi(vec_2)+Xi_add(vec_2_add);

end

