function [ vec_out ] = Hash_Org2Alpha( HashTable_Idx, idx_org )
%HASH_ORIGIN2BLK Summary of this function goes here
%   find the block index from origin index

idx_tmp = find(HashTable_Idx.idx_org == idx_org);

if numel(idx_tmp) ~= 1
    error('Hash Error!!!');
else
    idx_b = HashTable_Idx.idx_blk(idx_tmp)*3-2;
    idx_e = HashTable_Idx.idx_blk(idx_tmp)*3;
    vec_out = (idx_b:idx_e).';
end

end

