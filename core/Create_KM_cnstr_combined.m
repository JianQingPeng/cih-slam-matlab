function [ HashTable_Cnstr_SM0_ALL, Cnstr_Pool_Cell ] = Create_KM_cnstr_combined( JMInfo_SM0, JMInfo_Pool, HashTable_Dcmps, Cnstr_Pool_Cell)
%CREATE_KM_CNSTR_COMBINED Summary of this function goes here
%   Detailed explanation goes here
HashTable_Cnstr_SM0_ALL = JMInfo_SM0.HashTable_Cnstr;

num_submap = numel(JMInfo_Pool);

for i = 1:num_submap
    if isempty(JMInfo_Pool{i})
        continue;
    end
    HashTable_Cnstr_SM0 = JMInfo_Pool{i}.HashTable_Cnstr_SM0;
    idxvec_blk_key = JMInfo_Pool{i}.idxvec_blk_key;
    
    [rows, cols] = find(HashTable_Cnstr_SM0);
    
    for i2 = 1:numel(rows)
        
        idx_cnstr = HashTable_Cnstr_SM0(rows(i2), cols(i2));
        
        idx_blk_1 = idxvec_blk_key(rows(i2));
        idx_blk_2 = idxvec_blk_key(cols(i2));
        
        idx_local_blk_1 = HashTable_Dcmps.idx_local_blk(idx_blk_1);
        idx_local_blk_2 = HashTable_Dcmps.idx_local_blk(idx_blk_2);
        
        if HashTable_Cnstr_SM0_ALL(idx_local_blk_1,idx_local_blk_2) == 0
            HashTable_Cnstr_SM0_ALL(idx_local_blk_1,idx_local_blk_2) = idx_cnstr;
        else
            idx_cnstr_1 = idx_cnstr;
            idx_cnstr_2 = HashTable_Cnstr_SM0_ALL(idx_local_blk_1,idx_local_blk_2);
            I1 = Cnstr_Pool_Cell{idx_cnstr_1}.I;
            e1 = Cnstr_Pool_Cell{idx_cnstr_1}.e;
            I2 = Cnstr_Pool_Cell{idx_cnstr_2}.I;
            e2 = Cnstr_Pool_Cell{idx_cnstr_2}.e;
            [I3, e3] = Fun_Merge_InfoMatVec( I1, e1, I2, e2 );
            Cnstr_tmp.I = I3;
            Cnstr_tmp.e = e3;
            Cnstr_Pool_Cell{end+1} = Cnstr_tmp;
            HashTable_Cnstr_SM0_ALL(idx_local_blk_1,idx_local_blk_2) = numel(Cnstr_Pool_Cell);            
        end        
    end
end

