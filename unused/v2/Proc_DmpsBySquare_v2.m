function [ HashTable_Dcmps ] = Proc_DmpsBySquare_v2( Alpha, HashTable_Idx, HashTable_Cnstr, Cnstr_Pool)
%PROC_DMPSBYCIRCLE Summary of this function goes here
%   Detailed explanation goes here
HashTable_Dcmps = struct('idx_org',0,'idx_blk',1,'idx_submap',1,'idx_local_blk',[],'idx_local_b',[],'idx_local_e',[]);
DcmpsInfo = struct('idx_submap',1,'x_center',0,'y_center',0,'diameter',5);
x_center_now = 0;
y_center_now = 0;
diameter = 15;
idx_submap_now = 1;

%% set submap info of robot poses
vec_tmp = find(HashTable_Idx.idx_origin <= 0);
idxvec_odo_origin = HashTable_Idx.idx_origin(vec_tmp);
for i = idxvec_odo_origin(2:end)
    [x_tmp, y_tmp] = Pt_OriginIdx( Alpha, i, HashTable_Idx );
    dx = x_tmp - x_center_now;
    dy = y_tmp - y_center_now;
    HashTable_Dcmps.idx_org = [HashTable_Dcmps.idx_org; i];
    idx_blk_i = Hash_Origin2Blk(HashTable_Idx, i);
    HashTable_Dcmps.idx_blk = [HashTable_Dcmps.idx_blk; idx_blk_i];
    if (abs(dx) <= diameter/2) && (abs(dy) <= diameter/2)
        HashTable_Dcmps.idx_submap = [HashTable_Dcmps.idx_submap; idx_submap_now];
    else
        FindLoopClosure = false;
        dist2j_min = diameter/2;
        jMin = 0;
        x_center_jMin = 0;
        y_center_jMin = 0;
        for j = 1:DcmpsInfo.idx_submap(end)
            x_center_j = DcmpsInfo.x_center(j);
            y_center_j = DcmpsInfo.y_center(j);
            dx = x_tmp - x_center_j;
            dy = y_tmp - y_center_j;
            dist2center = max(abs(dx), abs(dy));
            if dist2center < dist2j_min
                dist2j_min = dist2center;
                jMin = j;
                x_center_jMin = x_center_j;
                y_center_jMin = y_center_j;
                FindLoopClosure = true;
            end
        end
        if FindLoopClosure == true
            HashTable_Dcmps.idx_submap = [HashTable_Dcmps.idx_submap; jMin];
            idx_submap_now = jMin;
            x_center_now = x_center_jMin;
            y_center_now = y_center_jMin;
        else
            idx_submap_now = DcmpsInfo.idx_submap(end)+1;
            HashTable_Dcmps.idx_submap = [HashTable_Dcmps.idx_submap; idx_submap_now];
            x_center_now = floor((x_tmp+diameter/2)/diameter)*diameter;
            y_center_now = floor((y_tmp+diameter/2)/diameter)*diameter;
            DcmpsInfo.idx_submap = [DcmpsInfo.idx_submap;idx_submap_now];
            DcmpsInfo.x_center = [DcmpsInfo.x_center; x_center_now];
            DcmpsInfo.y_center = [DcmpsInfo.y_center; y_center_now];
            DcmpsInfo.diameter = [DcmpsInfo.diameter; diameter];
        end
    end
end

%% set submap info of features
vec_tmp = find(HashTable_Idx.idx_origin > 0);
idxvec_feature_origin = HashTable_Idx.idx_origin(vec_tmp);
for i = idxvec_feature_origin
    idx_org_feature = i;
    idx_blk_feature = Hash_Origin2Blk(HashTable_Idx, idx_org_feature);
    idx_blk_odo = find(HashTable_Cnstr(:,idx_blk_feature),1);
    idx_org_odo = Hash_Blk2Origin(HashTable_Idx, idx_blk_odo);
    idx_submap = Hash_Org2Submap(HashTable_Dcmps, idx_org_odo);
    
    HashTable_Dcmps.idx_org = [HashTable_Dcmps.idx_org; idx_org_feature];
    HashTable_Dcmps.idx_submap = [HashTable_Dcmps.idx_submap; idx_submap];
    HashTable_Dcmps.idx_blk = [HashTable_Dcmps.idx_blk; idx_blk_feature];
end

%% set keynodes
num_submap = DcmpsInfo.idx_submap(end);
for i = 1:num_submap-1
    vec_tmp = find(HashTable_Dcmps.idx_submap == i);
    vec_org_i = HashTable_Dcmps.idx_org(vec_tmp);
    vec_blk_i = HashTable_Dcmps.idx_blk(vec_tmp);
    for j = i+1:num_submap
        vec_tmp = find(HashTable_Dcmps.idx_submap == j);
        vec_org_j = HashTable_Dcmps.idx_org(vec_tmp);
        vec_blk_j = HashTable_Dcmps.idx_blk(vec_tmp);        
       
        HashTable_Cnstr_ij = HashTable_Cnstr(vec_blk_i, vec_blk_j);
        HashTable_Cnstr_ji = HashTable_Cnstr(vec_blk_j, vec_blk_i);
        
%         %% set ij keynodes
%         [rows, cols] = find(HashTable_Cnstr_ij);
%         numel_cnstr_ij = numel(rows);
%         for k = 1:numel_cnstr_ij
%             idx_cnstr_tmp = HashTable_Cnstr_ij(rows(k), cols(k));
%             idx_blk_1_tmp = Cnstr_Pool(idx_cnstr_tmp).idx_blk_1;
%             idx_blk_2_tmp = Cnstr_Pool(idx_cnstr_tmp).idx_blk_2;            
%             idx_org_1 = Hash_Blk2Origin(HashTable_Idx, idx_blk_1_tmp);
%             idx_org_2 = Hash_Blk2Origin(HashTable_Idx, idx_blk_2_tmp);            
%             idx_org_key_tmp = min(idx_org_1, idx_org_2);           
%             idx_tmp = find(HashTable_Dcmps.idx_org == idx_org_key_tmp);
%             HashTable_Dcmps.idx_submap(idx_tmp) = 0;
%         end
%        %% set ji keynodes        
%         [rows, cols] = find(HashTable_Cnstr_ji);
%         numel_cnstr_ji = numel(rows);
%         for k = 1:numel_cnstr_ji
%             idx_cnstr_tmp = HashTable_Cnstr_ji(rows(k), cols(k));
%             idx_blk_1_tmp = Cnstr_Pool(idx_cnstr_tmp).idx_blk_1;
%             idx_blk_2_tmp = Cnstr_Pool(idx_cnstr_tmp).idx_blk_2;            
%             idx_org_1 = Hash_Blk2Origin(HashTable_Idx, idx_blk_1_tmp);
%             idx_org_2 = Hash_Blk2Origin(HashTable_Idx, idx_blk_2_tmp);            
%             idx_org_key_tmp = min(idx_org_1, idx_org_2);
%             idx_tmp = find(HashTable_Dcmps.idx_org == idx_org_key_tmp);
%             HashTable_Dcmps.idx_submap(idx_tmp) = 0;
%         end
        
        %% new algorithm, find least key nodes
        [ vec_i, vec_j ] = FindLeastKeyNodes( HashTable_Cnstr_ij, HashTable_Cnstr_ji );
        vec_blk_key = [vec_blk_i(vec_i);vec_blk_j(vec_j)];
        for k = 1:numel(vec_blk_key)
            vec_tmp = find(HashTable_Dcmps.idx_blk == vec_blk_key(k));
            HashTable_Dcmps.idx_submap(vec_tmp) = 0;
        end
        
    end
end
idx_tmp = find(HashTable_Dcmps.idx_org == 0);
HashTable_Dcmps.idx_submap(idx_tmp) = 0;

%% set local id
HashTable_Dcmps.idx_local_blk = zeros(size(HashTable_Dcmps.idx_org));
num_submap = DcmpsInfo.idx_submap(end) + 1;

DcmpsInfo.idx_submap = [DcmpsInfo.idx_submap; 0];
DcmpsInfo.x_center = [DcmpsInfo.x_center; 0];
DcmpsInfo.y_center = [DcmpsInfo.y_center; 0];
DcmpsInfo.diameter = [DcmpsInfo.diameter; -1];
DcmpsInfo.idxvec_org = cell(num_submap,1);
DcmpsInfo.idxvec_blk = cell(num_submap,1);

for i = 1:num_submap
    idx_submap = DcmpsInfo.idx_submap(i);
    vec_tmp = find(HashTable_Dcmps.idx_submap == idx_submap);
    num_local = numel(vec_tmp);
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
        if idx_org_tmp <= 0     % pose node
            HashTable_Dcmps.idx_local_e(idx_tmp,1) = HashTable_Dcmps.idx_local_b(idx_tmp)+2;
        else                    % feature node
            HashTable_Dcmps.idx_local_e(idx_tmp,1) = HashTable_Dcmps.idx_local_b(idx_tmp)+1;
        end
    end
    
    
    idxvec_org = HashTable_Dcmps.idx_org(vec_tmp);
    idxvec_blk = HashTable_Dcmps.idx_blk(vec_tmp);
    DcmpsInfo.idxvec_org{i,1} = idxvec_org;  
    DcmpsInfo.idxvec_blk{i,1} = idxvec_blk;  
end

