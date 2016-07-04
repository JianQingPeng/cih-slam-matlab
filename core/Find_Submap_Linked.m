function [ idxvec_submap ] = Find_Submap_Linked( ...
    idx_blk_ref, idx_blk_last, HashTable_Cnstr, HashTable_Dcmps )
%FIND_SUBMAP_LINKED Summary of this function goes here
%   find the index of all submap which linked to the reference node (idx_blk_ref)
idxvec_submap = [];
idxvec = (1:idx_blk_last).';
HT_1 = HashTable_Cnstr(idxvec,idx_blk_ref);
HT_2 = HashTable_Cnstr(idx_blk_ref,idxvec).';
idxvec_link = find(HT_1+HT_2);
idxvec_submap = unique(HashTable_Dcmps.idx_submap(idxvec_link));
end

