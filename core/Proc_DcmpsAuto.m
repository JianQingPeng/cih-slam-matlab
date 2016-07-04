function [ HashTable_Dcmps, G_DcmpsN ] = Proc_DcmpsAuto( HashTable_Idx, HashTable_Cnstr )
%PROC_DCMPSAUTO: graph decompose with Ncut approaximate algorithm

%% graph segment test
num_nodes = numel(HashTable_Idx.idx_blk);

W = boolean(HashTable_Cnstr);
W = W+W.';
W = double(W);
sz = size(W);
I = 1:sz(1);
sNcut = 0.5;
sArea = 50;
id = [];
[Seg Id Ncut] = NcutPartition(I, W, sNcut, sArea, id);

HashTable_Dcmps.idx_org = HashTable_Idx.idx_org;
HashTable_Dcmps.idx_blk = HashTable_Idx.idx_blk;
HashTable_Dcmps.idx_submap = zeros(num_nodes, 1);

num_submap = numel(Seg);
for i = 1:num_submap
    HashTable_Dcmps.idx_submap(Seg{i}) = i;
end

%% set key nodes
G_DcmpsN = sparse(num_submap,num_submap);
for i = 1:num_submap-1
    vec_tmp = Seg{i};
    vec_blk_i = HashTable_Dcmps.idx_blk(vec_tmp);
    for j = i+1:num_submap
        vec_tmp = Seg{j};
        vec_blk_j = HashTable_Dcmps.idx_blk(vec_tmp);
        
        HashTable_Cnstr_ij = HashTable_Cnstr(vec_blk_i, vec_blk_j);
        HashTable_Cnstr_ji = HashTable_Cnstr(vec_blk_j, vec_blk_i);
        
        %% find least key nodes
        [ vec_i, vec_j ] = FindLeastKeyNodes( HashTable_Cnstr_ij, HashTable_Cnstr_ji );
        vec_blk_key = [];
        if numel(vec_i) > 0 && numel(vec_j) > 0
            vec_blk_key = [vec_blk_i(vec_i);vec_blk_j(vec_j)];
        elseif numel(vec_i) > 0
            vec_blk_key = [vec_blk_i(vec_i)];
        elseif numel(vec_j) > 0
            vec_blk_key = [vec_blk_j(vec_j)];
        end
        
        for k = 1:numel(vec_blk_key)
            HashTable_Dcmps.idx_submap(vec_blk_key(k)) = 0;
        end
        if ~isempty(vec_blk_key)
            G_DcmpsN(i,j) = 1;
            G_DcmpsN(j,i) = 1;

        end
    end
end
idx_tmp = find(HashTable_Dcmps.idx_org == 0);
HashTable_Dcmps.idx_submap(idx_tmp) = 0;

%% set local info
HashTable_Dcmps.idx_local_blk = zeros(size(HashTable_Dcmps.idx_org));

for i = 0:num_submap
    idx_submap = i;
    vec_tmp = find(HashTable_Dcmps.idx_submap == idx_submap);
    num_local = numel(vec_tmp);
    
    if num_local == 0
        continue;
    end
    
    HashTable_Dcmps.idx_local_blk(vec_tmp) = (1:num_local).';
    
    for j = 1:num_local
        idx_tmp = vec_tmp(j);
        idx_org_tmp = HashTable_Dcmps.idx_org(idx_tmp);
        if j == 1;
            HashTable_Dcmps.idx_local_b(idx_tmp,1) = 1;
        else
            idx_tmp_last = vec_tmp(j-1);
            HashTable_Dcmps.idx_local_b(idx_tmp,1) = HashTable_Dcmps.idx_local_e(idx_tmp_last)+1;
        end
        HashTable_Dcmps.idx_local_e(idx_tmp,1) = HashTable_Dcmps.idx_local_b(idx_tmp)+2;
    end
end
end

