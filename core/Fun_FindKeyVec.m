function [ idxvec_blk_key ] = Fun_FindKeyVec( idxvec_blk_input, HashTable_Dcmps )
%FUN_FINDKEYVEC Summary of this function goes here
%   Find the idx_blk of all key nodes from a index vector
num = numel(idxvec_blk_input);
idxvec_blk_key = [];
for i = 1:num
    idx_blk_tmp = idxvec_blk_input(i);
    idx_submap_tmp = HashTable_Dcmps.idx_submap(idx_blk_tmp);
    if idx_submap_tmp == 0
        idxvec_blk_key = [idxvec_blk_key; idx_blk_tmp];
    end
end
end

