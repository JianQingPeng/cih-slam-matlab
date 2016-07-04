function [ alpha ] = Proc_InitAlpha_2( HashTable_Cnstr, Cnstr_Pool, alpha_init, idx_blk_init )
%PROC_INITALPHA_2: Init nodes vector Alpha, version 2
% find shortest spanning tree
% alpha_init: initial set of first several nodes

sz = size(HashTable_Cnstr);
dim = sz(1);
alpha = sparse(3*dim, 1);
G = boolean(HashTable_Cnstr+HashTable_Cnstr.');
[S, C] = graphconncomp(G);
if S > 1
    for i = 1:S
        idxvec_blk_sub{i} = find(C == i);
        alpha_init_sub{i} = [];
        idx_blk_init_sub{i} = [];
    end
    
    for j = 1:numel(idx_blk_init)
        alpha_tmp = alpha_init(j*3-2:j*3);
        idx_sub_tmp = C(idx_blk_init(j));
        idxvec_blk_sub_tmp = idxvec_blk_sub{idx_sub_tmp};
        idx_local_tmp = find(idxvec_blk_sub_tmp == idx_blk_init(j));
        idx_blk_init_sub{idx_sub_tmp} = [idx_blk_init_sub{idx_sub_tmp}; idx_local_tmp];
        alpha_init_sub{idx_sub_tmp} = [alpha_init_sub{idx_sub_tmp}; alpha_tmp];
    end
    
    for i = 1:S
        HashTable_Cnstr_tmp = HashTable_Cnstr(idxvec_blk_sub{i}, idxvec_blk_sub{i});
        alpha_init_tmp = alpha_init_sub{i};
        idx_blk_init_tmp = idx_blk_init_sub{i};
        alpha_tmp = Proc_InitAlpha_2( HashTable_Cnstr_tmp, Cnstr_Pool, alpha_init_tmp, idx_blk_init_tmp );
        
        idxvec_alpha_tmp = Hash_Blk2Alpha([], idxvec_blk_sub{i});
        alpha(idxvec_alpha_tmp) = alpha_tmp;
    end
else
    idx_blk_start = idx_blk_init(1);
    [disc, pred] = graphtraverse(G, idx_blk_start);
    
    idxvec_alpha_init = Hash_Blk2Alpha([], idx_blk_init);
    alpha(idxvec_alpha_init) = alpha_init;
    
    for i = 2:numel(disc)
        
        idx_blk_2 = disc(i);
        idx_blk_1 = pred(idx_blk_2);
        
        vec_alpha_1 = alpha(idx_blk_1*3-2:idx_blk_1*3);
        T1 = Trans_Mat_Pose(vec_alpha_1);
        
        idx_cnstr_1 = HashTable_Cnstr(idx_blk_1,idx_blk_2);
        idx_cnstr_2 = HashTable_Cnstr(idx_blk_2,idx_blk_1);
        
        if idx_cnstr_1 ~= 0
            vec_12 = Cnstr_Pool{idx_cnstr_1}.e;
            T_12 = Trans_Mat_Pose(vec_12);
            T2 = T1*T_12;
            vec_alpha_2 = Trans_Mat_Pose(T2);
        else
            vec_21 = Cnstr_Pool{idx_cnstr_2}.e;
            T_21 = Trans_Mat_Pose(vec_21);
            T2 = T1*inv(T_21);
            vec_alpha_2 = Trans_Mat_Pose(T2);
        end
        alpha(idx_blk_2*3-2:idx_blk_2*3) = vec_alpha_2;
    end
    
    idxvec_alpha_init = Hash_Blk2Alpha([], idx_blk_init);
    alpha(idxvec_alpha_init) = alpha_init;
    
end

end

