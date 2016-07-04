function [ vec_i, vec_j ] = FindLeastKeyNodes( HashTable_Cnstr_ij, HashTable_Cnstr_ji )
%FINDLEASTKEYNODES Summary of this function goes here
%   Detailed explanation goes here
Hash_ij_cmb = HashTable_Cnstr_ij + HashTable_Cnstr_ji.';
vec_i = []; vec_j = [];
while nnz(Hash_ij_cmb) ~= 0
    sz = size(Hash_ij_cmb);
    if_i = false;
    if_j = false;
    nnz_now = 0;
    idx_now = -1;
    for i = 1:sz(1)
        nnz_tmp = nnz(Hash_ij_cmb(i,:));
        if nnz_tmp > nnz_now
            nnz_now = nnz_tmp;
            if_i = true;
            if_j = false;
            idx_now = i;
        end
    end
    for j = 1:sz(2)
        nnz_tmp = nnz(Hash_ij_cmb(:,j));
        if nnz_tmp > nnz_now
            nnz_now = nnz_tmp;
            if_i = false;
            if_j = true;
            idx_now = j;
        end
    end
    
    if if_i
        vec_i = [vec_i; idx_now];
        sz_tmp = size(Hash_ij_cmb(idx_now,:));
        Hash_ij_cmb(idx_now,:) = zeros(sz_tmp);
    elseif if_j
        vec_j = [vec_j; idx_now];
        sz_tmp = size(Hash_ij_cmb(:,idx_now));
        Hash_ij_cmb(:,idx_now) = zeros(sz_tmp);
    end        
end

end

