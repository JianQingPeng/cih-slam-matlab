function [ Alpha ] = Solve_NormalGr_v2(Alpha, Alpha_key, HashTable_Idx, HashTable_Dcmps, JointGraph_Pool, SingleGraph_Pool)
%SOLVE_NORMALGR_V2 Summary of this function goes here
%   Detailed explanation goes here
%% get key nodes info
num_submap = max(HashTable_Dcmps.idx_submap);
vec_tmp = find(HashTable_Dcmps.idx_submap == 0);
vec_local_e_k = HashTable_Dcmps.idx_local_e(vec_tmp);
vec_blk_k = HashTable_Dcmps.idx_blk(vec_tmp);
idx_local_e_max = max(vec_local_e_k);
IdxVec_key = (1:idx_local_e_max);
%% set key node into alpha
vec_alpha_k = [];
for k = 1:numel(vec_blk_k)
    vec_tmp = Hash_Blk2Alpha(HashTable_Idx ,vec_blk_k(k));
    vec_alpha_k = [vec_alpha_k; vec_tmp];
end
Alpha(vec_alpha_k) = Alpha_key;

%% for each normal sub map
for j = 1:num_submap
    JointGraph_Pool(j).Omega;
    if numel(JointGraph_Pool(j).Xi) == 0
        continue;
    end
    %% solve sub map
    Omega_kn = JointGraph_Pool(j).Omega;
    Omega_kn(idx_local_e_max+1:end, idx_local_e_max+1:end) = ...
        Omega_kn(idx_local_e_max+1:end, idx_local_e_max+1:end) + SingleGraph_Pool(j).Omega;
    Xi_kn = JointGraph_Pool(j).Xi;
    Xi_kn(idx_local_e_max+1:end) = Xi_kn(idx_local_e_max+1:end) + SingleGraph_Pool(j).Xi;
    [ Omega_n, Xi_n ] = Cal_InfoMatVec_Conditional_v2( ...
        Omega_kn, Xi_kn, Alpha_key, IdxVec_key );
    Omega_n_sp = sparse(Omega_n);
    Xi_n_sp = sparse(Xi_n);
    Alpha_n_sp = Omega_n_sp\Xi_n_sp;
    Alpha_n = full(Alpha_n_sp);
    %% set into Alpha
    vec_tmp = find(HashTable_Dcmps.idx_submap == j);
    vec_blk_n = HashTable_Dcmps.idx_blk(vec_tmp);
    vec_alpha_n = [];
    for k = 1:numel(vec_blk_n)
        vec_tmp = Hash_Blk2Alpha(HashTable_Idx, vec_blk_n(k));
        vec_alpha_n = [vec_alpha_n; vec_tmp];
    end
    Alpha(vec_alpha_n) = Alpha_n;
end

end

