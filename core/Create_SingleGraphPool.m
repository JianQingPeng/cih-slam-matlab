function [ SingleGraph_Pool ] = Create_SingleGraphPool( ...
    Alpha, HashTable_Idx, HashTable_Dcmps, HashTable_Cnstr, Cnstr_Pool )
%CREATE_SINGLEGRAPHPOOL Summary of this function goes here
%   Detailed explanation goes here
% set number of submaps
    num_submap = max(HashTable_Dcmps.idx_submap);
    SingleGraph_Info_tmp = struct('Omega',[],'Xi',[],'Alpha',[],'Idx_submap',[]);
    for j = 1:num_submap
        SingleGraph_Pool(j,1) = SingleGraph_Info_tmp;
    end
    for j = [1:num_submap, 0]
        % load info from HashTable_Dcmps
        idxvec_tmp = find(HashTable_Dcmps.idx_submap == j);
        idxvec_idx_org = HashTable_Dcmps.idx_org(idxvec_tmp);
        idxvec_idx_blk = HashTable_Dcmps.idx_blk(idxvec_tmp);
        idxvec_idx_local_blk = HashTable_Dcmps.idx_local_blk(idxvec_tmp);
        idxvec_idx_local_b = HashTable_Dcmps.idx_local_b(idxvec_tmp);
        idxvec_idx_local_e = HashTable_Dcmps.idx_local_e(idxvec_tmp);
        dim_now = max(idxvec_idx_local_e);
        
        % create empty Omega and Xi
        Omega_now = zeros(dim_now,dim_now);
        Xi_now = zeros(dim_now,1);
        
        % refresh Omega and Xi by all constraints within the subgraph
        HashTable_Cnstr_now = HashTable_Cnstr(idxvec_idx_blk,idxvec_idx_blk);
        [rows, cols] = find(HashTable_Cnstr_now);
        num_cnstr_now = numel(rows);
        for k = 1:num_cnstr_now
            % load constraint information
            idx_cnstr_tmp = HashTable_Cnstr_now(rows(k), cols(k));
            I_tmp = Cnstr_Pool(idx_cnstr_tmp).I;
            e_tmp = Cnstr_Pool(idx_cnstr_tmp).e;
            idx_blk_1_tmp = Cnstr_Pool(idx_cnstr_tmp).idx_blk_1;
            idx_blk_2_tmp = Cnstr_Pool(idx_cnstr_tmp).idx_blk_2;
            % calculate Omega and Xi for this constraint
%             vec_alpha_1 = Hash_Blk2Alpha( HashTable_Idx, idx_blk_1_tmp );
%             vec_alpha_2 = Hash_Blk2Alpha( HashTable_Idx, idx_blk_2_tmp );
            idx_tmp = find(HashTable_Idx.idx_blk == idx_blk_1_tmp);
            idx_b_1 = HashTable_Idx.idx_st_b(idx_tmp);
            idx_e_1 = HashTable_Idx.idx_st_e(idx_tmp);
            vec_alpha_1 = idx_b_1:idx_e_1;
            idx_tmp = find(HashTable_Idx.idx_blk == idx_blk_2_tmp);
            idx_b_2 = HashTable_Idx.idx_st_b(idx_tmp);
            idx_e_2 = HashTable_Idx.idx_st_e(idx_tmp);
            vec_alpha_2 = idx_b_2:idx_e_2;

            p1 = Alpha(vec_alpha_1);
            p2 = Alpha(vec_alpha_2);
            [ Omega_add, Xi_add ] = Cal_InfoMatVec_v2(p1, p2, I_tmp, e_tmp);
            
            % add into Omega and Xi
            idx_tmp_1 = find(HashTable_Dcmps.idx_blk == idx_blk_1_tmp);
            idx_tmp_2 = find(HashTable_Dcmps.idx_blk == idx_blk_2_tmp);
            
            idx_local_b_1 = HashTable_Dcmps.idx_local_b(idx_tmp_1);
            idx_local_e_1 = HashTable_Dcmps.idx_local_e(idx_tmp_1);
            idx_local_b_2 = HashTable_Dcmps.idx_local_b(idx_tmp_2);
            idx_local_e_2 = HashTable_Dcmps.idx_local_e(idx_tmp_2);
            vec_1 = (idx_local_b_1:idx_local_e_1);
            vec_2 = (idx_local_b_2:idx_local_e_2);
            
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
            
            Omega_now(vec_2,vec_1) = Omega_now(vec_2,vec_1) + Omega_add(vec_2_add,vec_1_add);
            Omega_now(vec_2,vec_2) = Omega_now(vec_2,vec_2) + Omega_add(vec_2_add,vec_2_add);
            Omega_now(vec_1,vec_1) = Omega_now(vec_1,vec_1) + Omega_add(vec_1_add,vec_1_add);
            Omega_now(vec_1,vec_2) = Omega_now(vec_1,vec_2) + Omega_add(vec_1_add,vec_2_add);
            
            Xi_now(vec_1) = Xi_now(vec_1)+Xi_add(vec_1_add);
            Xi_now(vec_2) = Xi_now(vec_2)+Xi_add(vec_2_add);
        end
        
        if j ~= 0
            SingleGraph_Info_tmp = struct('Omega',[],'Xi',[],'Alpha',[],'Idx_submap',[]);
            SingleGraph_Info_tmp.Omega = Omega_now;
            SingleGraph_Info_tmp.Xi = Xi_now;
            SingleGraph_Info_tmp.Idx_submap = j;
            SingleGraph_Pool(j,1) = SingleGraph_Info_tmp;
        else
            SingleGraph_Info_tmp = struct('Omega',[],'Xi',[],'Alpha',[],'Idx_submap',[]);
            SingleGraph_Info_tmp.Omega = Omega_now;
            SingleGraph_Info_tmp.Xi = Xi_now;
            SingleGraph_Info_tmp.Idx_submap = j;
            SingleGraph_Pool(num_submap+1,1) = SingleGraph_Info_tmp;
        end
    end

end

