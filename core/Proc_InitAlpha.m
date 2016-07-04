function [ alpha ] = Proc_InitAlpha( HashTable_Idx, HashTable_Cnstr, Cnstr_Pool )
%PROC_INITALPHA: Init nodes vector Alpha with naive algorithm
% not based on tree
% no init status set, Alpha(1:3) = [0;0;0] by default

idx_blk_initialized = 1;
num_pose = numel(HashTable_Idx.idx_org);
alpha = zeros(num_pose*3,1);
% to sort by rows.
[cols, rows] = find(HashTable_Cnstr.');
num_cnstr = numel(rows);
for i = 1:num_cnstr
    idx_blk_1 = rows(i);
    idx_blk_2 = cols(i);   
    
    if isempty(find(idx_blk_initialized == idx_blk_1))
        error('Error in Initialization!!!');
    end
    
    idxvec_1 = HashTable_Idx.idx_blk(idx_blk_1)*3-2:HashTable_Idx.idx_blk(idx_blk_1)*3;
    idxvec_2 = HashTable_Idx.idx_blk(idx_blk_2)*3-2:HashTable_Idx.idx_blk(idx_blk_2)*3;
    vec_1 = alpha(idxvec_1);
    
    if isempty(find(idx_blk_initialized == idx_blk_2))
        idx_cnstr = HashTable_Cnstr(idx_blk_1,idx_blk_2);
        e_tmp = Cnstr_Pool{idx_cnstr}.e;
        R1 = Trans_RotMat_Angle(vec_1(3));
        
        if numel(e_tmp) == 2            
            vec_2 = vec_1(1:2) + R1*e_tmp;
        elseif numel(e_tmp) == 3
            vec_2 = vec_1 + blkdiag(R1,1)*e_tmp;
        else
            error('Error in Initialization!!!');
        end
        
        alpha(idxvec_2) = vec_2;
        idx_blk_initialized = [idx_blk_initialized;idx_blk_2]; 
    end
end

end

