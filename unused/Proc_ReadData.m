function [ HashTable_Idx, HashTable_Cnstr, Cnstr_Pool ] = Proc_ReadData( dataFold_path, num_localMaps, ReadStatus_IfPoseNegative )
%PROC_READDATA_LM Summary of this function goes here
%   Detailed explanation goes here

% create hash table for index
% idx_origion: index from input data, positive for feature, 0 or negative
% for pose
% idx_blk: block index in Xi and Omega
% idx_st_b: element index in Xi, begin index
% idx_st_e: element index in Xi, end index
HashTable_Idx = struct('idx_org', 0, 'idx_blk', 1);
% create constraints pool
% HashTable_Cnstr: hash table from block index to constraint pool
HashTable_Cnstr = sparse(10000,10000);
% Cnstr_Pool: constraint pool, save all the constraint info
% I: infomation matrix; e: measurement vector;
% idx_blk_1, idx_blk_2: block index for first and second node of this edge
Cnstr_Element_tmp = struct('I',[],'e',[]);
Cnstr_Pool{1} = Cnstr_Element_tmp;
for i = 1:num_localMaps
    dataFile_path = [dataFold_path, '/localmap_', int2str(i), '.mat' ];
    load(dataFile_path);
    if isempty(find(HashTable_Idx.idx_org == -Ref))
        HashTable_Idx.idx_org(end+1) = -Ref;
        HashTable_Idx.idx_blk(end+1) = HashTable_Idx.idx_blk(end) + 1;
    end
    Idx_blk_1_tmp = Hash_Org2Blk(HashTable_Idx, -Ref);
    [sz_localmap, tmp] = size(st);
    for j = 1:sz_localmap
        if ReadStatus_IfPoseNegative
            % for local map with feature
            idx_org_tmp = st(j,1);
        else
            % for local map without feature, database Intel
            idx_org_tmp = -st(j,1);
        end
        
        if isempty(find(HashTable_Idx.idx_org == idx_org_tmp))
            HashTable_Idx.idx_org(end+1) = idx_org_tmp;
            HashTable_Idx.idx_blk(end+1) = HashTable_Idx.idx_blk(end) + 1;
        end
        Idx_blk_2_tmp = Hash_Org2Blk(HashTable_Idx, idx_org_tmp);
        if (HashTable_Cnstr(Idx_blk_1_tmp,Idx_blk_2_tmp) == 0)
            HashTable_Cnstr(Idx_blk_1_tmp,Idx_blk_2_tmp) = numel(Cnstr_Pool);
            if idx_org_tmp > 0
                I_tmp = I(j:j+1, j:j+1);
                e_tmp = st(j:j+1, 2);
            else
                I_tmp = I(j:j+2, j:j+2);
                e_tmp = st(j:j+2, 2);
            end
            Cnstr_Element_tmp.I = I_tmp;
            Cnstr_Element_tmp.e = e_tmp;
            Cnstr_Pool{end+1} = Cnstr_Element_tmp;
        end
    end
end
Cnstr_Pool(1) = [];
num_node = numel(HashTable_Idx.idx_blk);
HashTable_Cnstr = HashTable_Cnstr(1:num_node, 1:num_node);

%% sort by robot pose index
[vec_tmp, idxvec_tmp] = sort(HashTable_Idx.idx_org, 'descend');
HashTable_Idx.idx_org = HashTable_Idx.idx_org(idxvec_tmp);
HashTable_Cnstr = HashTable_Cnstr(idxvec_tmp,idxvec_tmp);

end

