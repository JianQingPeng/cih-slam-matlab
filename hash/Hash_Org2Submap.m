function [ idx_submap ] = Hash_Org2Submap( HashTable_Dmps, idx_org )
%HASH_ORG2SUBMAP Summary of this function goes here
%   Detailed explanation goes here
idx_tmp = find(HashTable_Dmps.idx_org == idx_org);
idx_submap = HashTable_Dmps.idx_submap(idx_tmp);

end

