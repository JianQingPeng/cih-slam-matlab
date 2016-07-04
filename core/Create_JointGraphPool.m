function [ JointGraph_Pool ] = Create_JointGraphPool...
    ( Alpha, HashTable_Idx, HashTable_Dcmps, HashTable_Cnstr, Cnstr_Pool )
%CREATE_JOINTGRAPHPOOL Summary of this function goes here

%% init
% set number of submaps
num_submap = max(HashTable_Dcmps.idx_submap);

% create joint graph pool, for all the normal sub graph;
% joint with sub graph 0: the key node graph
JointGraph_Info_tmp = struct('Omega',[],'Xi',[],'Alpha',[],'Idx_submap',[]);
for j = 1:num_submap
    JointGraph_Pool(j,1) = JointGraph_Info_tmp;
end

%% init submap 0, with all key nodes
% set submap of key nodes, with submap index 0
idx_tmp = find(HashTable_Dcmps.idx_submap == 0);
idxvec_org_submap_k = HashTable_Dcmps.idx_org(idx_tmp);
idxvec_blk_submap_k = HashTable_Dcmps.idx_blk(idx_tmp);
idxvec_local_blk_k = HashTable_Dcmps.idx_local_blk(idx_tmp);
idxvec_local_b_k = HashTable_Dcmps.idx_local_b(idx_tmp);
idxvec_local_e_k = HashTable_Dcmps.idx_local_e(idx_tmp);
dim_k = max(idxvec_local_e_k);

% for all the other submap
for j = 1:num_submap
    idx_tmp = find(HashTable_Dcmps.idx_submap == j);
    idxvec_org_submap_n = HashTable_Dcmps.idx_org(idx_tmp);
    idxvec_blk_submap_n = HashTable_Dcmps.idx_blk(idx_tmp);
    idxvec_local_blk_n = HashTable_Dcmps.idx_local_blk(idx_tmp);
    idxvec_local_b_n = HashTable_Dcmps.idx_local_b(idx_tmp);
    idxvec_local_e_n = HashTable_Dcmps.idx_local_e(idx_tmp);
    dim_n = max(idxvec_local_e_n);
    
    Omega_kn = zeros(dim_k+dim_n, dim_k+dim_n);
    Xi_kn = zeros(dim_k+dim_n, 1);
    
    HashTable_Cnstr_nk = HashTable_Cnstr(idxvec_blk_submap_n, idxvec_blk_submap_k);
    HashTable_Cnstr_kn = HashTable_Cnstr(idxvec_blk_submap_k, idxvec_blk_submap_n);
    
    [n1, k1] = find(HashTable_Cnstr_nk);
    idxvec_cnstr_1 = [];
    if numel(n1) > 0
        for k = 1:numel(n1)
            idxvec_cnstr_tmp = HashTable_Cnstr_nk(n1(k), k1(k));
            idxvec_cnstr_1 = [idxvec_cnstr_1; idxvec_cnstr_tmp];
        end
    end
    [k2, n2] = find(HashTable_Cnstr_kn);
    idxvec_cnstr_2 = [];
    if numel(n2) > 0
        for k = 1:numel(n2)
            idxvec_cnstr_tmp = HashTable_Cnstr_kn(k2(k), n2(k));
            idxvec_cnstr_2 = [idxvec_cnstr_2; idxvec_cnstr_tmp];
        end
    end
    
    idxvec_cnstr = [idxvec_cnstr_1;idxvec_cnstr_2];
    % for each constraint
    for k = 1:numel(idxvec_cnstr)
        idx_cnstr = idxvec_cnstr(k);
        Cnstr_Info_tmp = Cnstr_Pool(idx_cnstr);
        I_tmp = Cnstr_Info_tmp.I;
        e_tmp = Cnstr_Info_tmp.e;
        idx_blk_1_tmp = Cnstr_Info_tmp.idx_blk_1;
        idx_blk_2_tmp = Cnstr_Info_tmp.idx_blk_2;
        
        idxvec_alpha_1 = Hash_Blk2Alpha(HashTable_Idx, idx_blk_1_tmp);
        idxvec_alpha_2 = Hash_Blk2Alpha(HashTable_Idx, idx_blk_2_tmp);
        
        p1_tmp = Alpha(idxvec_alpha_1);
        p2_tmp = Alpha(idxvec_alpha_2);
        
        [ Omega_tmp, Xi_tmp ] = Cal_InfoMatVec_v2(p1_tmp, p2_tmp, I_tmp, e_tmp);
        
        idx_n_1 = find(idxvec_blk_submap_n == idx_blk_1_tmp);
        idx_n_2 = find(idxvec_blk_submap_n == idx_blk_2_tmp);
        idx_k_1 = find(idxvec_blk_submap_k == idx_blk_1_tmp);
        idx_k_2 = find(idxvec_blk_submap_k == idx_blk_2_tmp);
        
        if numel(idx_n_1) == 1 && numel(idx_n_2) == 0 && ...
                numel(idx_k_1) == 0 && numel(idx_k_2) == 1
            idx_local_b_1 = idxvec_local_b_n(idx_n_1)+dim_k;
            idx_local_e_1 = idxvec_local_e_n(idx_n_1)+dim_k;
            vec_1_kn = (idx_local_b_1:idx_local_e_1).';
            idx_local_b_2 = idxvec_local_b_k(idx_k_2);
            idx_local_e_2 = idxvec_local_e_k(idx_k_2);
            vec_2_kn = (idx_local_b_2:idx_local_e_2).';
        elseif numel(idx_n_1) == 0 && numel(idx_n_2) == 1 && ...
                numel(idx_k_1) == 1 && numel(idx_k_2) == 0
            idx_local_b_2 = idxvec_local_b_n(idx_n_2)+dim_k;
            idx_local_e_2 = idxvec_local_e_n(idx_n_2)+dim_k;
            vec_2_kn = (idx_local_b_2:idx_local_e_2).';
            idx_local_b_1 = idxvec_local_b_k(idx_k_1);
            idx_local_e_1 = idxvec_local_e_k(idx_k_1);
            vec_1_kn = (idx_local_b_1:idx_local_e_1).';
        else
            error('Error in create joint graph!!!');
        end
        
        if numel(vec_1_kn) == 3
            Omega_kn(vec_1_kn, vec_1_kn) = Omega_kn(vec_1_kn, vec_1_kn) + Omega_tmp(1:3,1:3);
            Omega_kn(vec_2_kn, vec_2_kn) = Omega_kn(vec_2_kn, vec_2_kn) + Omega_tmp(4:end,4:end);
            Omega_kn(vec_1_kn, vec_2_kn) = Omega_kn(vec_1_kn, vec_2_kn) + Omega_tmp(1:3,4:end);
            Omega_kn(vec_2_kn, vec_1_kn) = Omega_kn(vec_2_kn, vec_1_kn) + Omega_tmp(4:end,1:3);
            Xi_kn(vec_1_kn) = Xi_kn(vec_1_kn) + Xi_tmp(1:3);
            Xi_kn(vec_2_kn) = Xi_kn(vec_2_kn) + Xi_tmp(4:end);
        else
            Omega_kn(vec_1_kn, vec_1_kn) = Omega_kn(vec_1_kn, vec_1_kn) + Omega_tmp(1:2,1:2);
            Omega_kn(vec_2_kn, vec_2_kn) = Omega_kn(vec_2_kn, vec_2_kn) + Omega_tmp(3:end,3:end);
            Omega_kn(vec_1_kn, vec_2_kn) = Omega_kn(vec_1_kn, vec_2_kn) + Omega_tmp(1:2,3:end);
            Omega_kn(vec_2_kn, vec_1_kn) = Omega_kn(vec_2_kn, vec_1_kn) + Omega_tmp(3:end,1:2);
            Xi_kn(vec_1_kn) = Xi_kn(vec_1_kn) + Xi_tmp(1:2);
            Xi_kn(vec_2_kn) = Xi_kn(vec_2_kn) + Xi_tmp(3:end);
        end
    end
    JointGraph_Info_tmp = struct('Omega',[],'Xi',[],'Alpha',[],'Idx_submap',[]);
    JointGraph_Info_tmp.Omega = Omega_kn;
    JointGraph_Info_tmp.Xi = Xi_kn;
    JointGraph_Info_tmp.Idx_submap = j;
    JointGraph_Pool(j) = JointGraph_Info_tmp;
end

end

