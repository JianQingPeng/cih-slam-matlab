function [ Omega_res, Xi_res ] = Cal_InfoMatVec_Marginal( Omega, Xi, IdxVec_key )
%CAL_INFOMATVEC_MARGINAL Summary of this function goes here
[dim,tmp] = size(Omega);
IdxVec_normal = 1:dim;

for i = 1:numel(IdxVec_key)
    idx_tmp = find(IdxVec_normal == IdxVec_key(i));
    IdxVec_normal(idx_tmp) = [];
end

% k: key nodes, n: norm nodes
Omega_kk = Omega(IdxVec_key, IdxVec_key);
Omega_kn = Omega(IdxVec_key, IdxVec_normal);
Omega_nk = Omega(IdxVec_normal, IdxVec_key);
Omega_nn = Omega(IdxVec_normal, IdxVec_normal);

Xi_k = Xi(IdxVec_key);
Xi_n = Xi(IdxVec_normal);

% Do marginalize
Omega_kn_sp = sparse(Omega_kn);
Omega_nk_sp = sparse(Omega_nk);
Omega_nn_sp = sparse(Omega_nn);
Omega_kk_sp = sparse(Omega_kk);


Omega_tmp_sp = Omega_kn_sp/Omega_nn_sp;
Omega_res_sp = Omega_kk_sp - Omega_tmp_sp*Omega_nk_sp;
Xi_res_sp = Xi_k - Omega_tmp_sp*Xi_n;
Omega_res_sp = 0.5*(Omega_res_sp+Omega_res_sp');

Xi_res = full(Xi_res_sp);
Omega_res = full(Omega_res_sp);
end