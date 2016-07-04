function [ idx_org ] = Hash_Blk2Org( HashTable_Idx, Idx_blk)
%HASH_BLK2ORIGIN Summary of this function goes here
%   Detailed explanation goes here

idx_tmp = find(HashTable_Idx.idx_blk == Idx_blk);

if numel(idx_tmp) ~= 1
    error('Hash Error!!!');
else
    idx_org = HashTable_Idx.idx_org(idx_tmp);
end

end

