function [ Omega_key, Xi_key, Alpha_key ] = Solve_KeyGr_v2( SingleGraph_Pool, JointGraph_Pool)
%SOLVE_KEYGR_V2 Summary of this function goes here
%   Detailed explanation goes here
Omega_key = SingleGraph_Pool(end).Omega;
Xi_key = SingleGraph_Pool(end).Xi;
dim_key = numel(Xi_key);
IdxVec_key = 1:dim_key;
for j = 1:numel(JointGraph_Pool)
    if isempty(SingleGraph_Pool(j).Omega)
        continue;
    end
    Omega_joint_j = JointGraph_Pool(j).Omega;
    Xi_joint_j = JointGraph_Pool(j).Xi;
    Omega_joint_j(dim_key+1:end, dim_key+1:end) = ...
        Omega_joint_j(dim_key+1:end, dim_key+1:end) + SingleGraph_Pool(j).Omega;
    Xi_joint_j(dim_key+1:end,1) = Xi_joint_j(dim_key+1:end,1) + SingleGraph_Pool(j).Xi;
    
    [Omega_key_add, Xi_key_add] = Cal_InfoMatVec_Marginal_v2(Omega_joint_j, Xi_joint_j, IdxVec_key );
    Omega_key = Omega_key + Omega_key_add;
    Xi_key = Xi_key + Xi_key_add;
end

Alpha_key = Omega_key\Xi_key;
% plot(Alpha_key(1:3:end),Alpha_key(2:3:end));



%%%%%% debug code here, compare with Omega and Xi from Graph slam %%%%%%
%     load('matlab.mat');
%     vec_tmp = find(HashTable_Dcmps.idx_submap == 0);
%     vec_org = HashTable_Dcmps.idx_org(vec_tmp);
%     vec_alpha_key = [];
%     for j = 1:numel(vec_org)
%         vec_alpha_tmp = Hash_Origin2Alpha(HashTable_Idx, vec_org(j));
%         vec_alpha_key = [vec_alpha_key; vec_alpha_tmp];
%     end
%     [Omega_key_db, Xi_key_db] = Cal_InfoMatVec_Marginal_v2(Omega, Xi, vec_alpha_key);
%     Alpha_key_db = Omega_key_db\Xi_key_db;
%     plot(Alpha_key_db(1:3:end),Alpha_key_db(2:3:end));
%%%%%% debug code end %%%%%%

end