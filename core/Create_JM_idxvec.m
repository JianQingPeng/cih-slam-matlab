function [ idxvec_blk_JM, idxvec_blk_key, idxvec_blk_norm ] = Create_JM_idxvec( ...
    HashTable_Dcmps, HashTable_Cnstr, idx_blk_last, idx_submap )
%CREATE_JM_IDXVEC Summary of this function goes here
%   find the index vector of Joint Map


idxvec_blk_norm = find(HashTable_Dcmps.idx_submap(1:idx_blk_last) == idx_submap);
idxvec_blk_key = find(HashTable_Dcmps.idx_submap(1:idx_blk_last) == 0);

HashTable_Cnstr_1 = HashTable_Cnstr(idxvec_blk_key, idxvec_blk_norm);
HashTable_Cnstr_2 = HashTable_Cnstr(idxvec_blk_norm, idxvec_blk_key);

num_key_all = numel(idxvec_blk_key);

vec_toDelet = [];
for i = 1:num_key_all
    num_cnstr_tmp = nnz(HashTable_Cnstr_1(i,:)) + nnz(HashTable_Cnstr_2(:,i));
    if num_cnstr_tmp == 0
       vec_toDelet = [vec_toDelet; i];
    end 
end
idxvec_blk_key(vec_toDelet) = [];
idxvec_blk_JM = [idxvec_blk_key;idxvec_blk_norm];