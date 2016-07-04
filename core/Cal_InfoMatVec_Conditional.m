function [ Omega_out, Xi_out ] = Cal_InfoMatVec_Conditional( Omega, Xi, Alpha_k, IdxVec_k )
%CAL_INFOMATVEC_CONDITIONAL Summary of this function goes here
%   Detailed explanation goes here

num_k = numel(IdxVec_k);
dim = numel(Xi);
IdxVec_n = (1:dim).';

for i = 1:num_k
    idx_tmp = find(IdxVec_n == IdxVec_k(i));    
    IdxVec_n(idx_tmp) = []; 
end

% n: normal nodes
% k: key nodes need to be conditioned
Omega_nn = Omega(IdxVec_n,IdxVec_n);
Omega_nk = Omega(IdxVec_n,IdxVec_k);

Omega_out = Omega_nn;
Xi_out = Xi(IdxVec_n) - Omega_nk*Alpha_k;

end

