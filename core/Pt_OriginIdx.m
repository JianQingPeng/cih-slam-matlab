function [ x, y ] = Pt_OriginIdx( Alpha, idx_origin, HashTable_Idx )
%PT_ORIGINIDX Summary of this function goes here
%   return the cordinate of the nodes from its origin index

idxvec_alpha = Hash_Org2Alpha(HashTable_Idx, idx_origin);
vec_alpha = Alpha(idxvec_alpha);
x = vec_alpha(1);
y = vec_alpha(2);





end

