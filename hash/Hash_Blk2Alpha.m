function [ vec_out ] = Hash_Blk2Alpha( HashTable_Idx, idx_blk )
%HASH_BLK2ALPHA Summary of this function goes here
%   Detailed explanation goes here

if numel(idx_blk) == 1
    idx_b = idx_blk*3-2;
    idx_e = idx_blk*3;
    vec_out = (idx_b:idx_e).';
else
    vec_blk = idx_blk;
    sz = size(vec_blk);
    if sz(2) ~= 1
        vec_blk = vec_blk.';
    end
    vec_out_1 = vec_blk*3-2;
    vec_out_2 = vec_blk*3-1;
    vec_out_3 = vec_blk*3;
    vec_out = sort([vec_out_1;vec_out_2;vec_out_3]); 
end
    
    
end

