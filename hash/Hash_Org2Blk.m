function [ Idx_blk ] = Hash_Org2Blk( HashTable_Idx, idx_org )
%HASH_ORIGIN2BLK Summary of this function goes here
%   find the block index from origin index

idx_tmp = find(HashTable_Idx.idx_org == idx_org);

if numel(idx_tmp) ~= 1
    error('Hash Error!!!');
else
    Idx_blk = HashTable_Idx.idx_blk(idx_tmp);
end

end

