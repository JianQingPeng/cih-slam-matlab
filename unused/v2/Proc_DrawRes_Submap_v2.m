function Proc_DrawRes_Submap_v2( Alpha, HashTable_Idx, HashTable_Cnstr, HashTable_Dcmps)
%PROC_DRAWRES_V2 Summary of this function goes here
%   Detailed explanation goes here
figure;
grid on;
hold on;
axis equal;
% set(gca,'XTick',-1000:5:1000);
% set(gca,'YTick',-1000:5:1000);

% draw observation relations
[rows_tmp, cols_tmp] = find(HashTable_Cnstr);
num_cnstr = numel(rows_tmp);
for i = 1:num_cnstr
    idx_blk_1 = rows_tmp(i);
    idx_blk_2 = cols_tmp(i);
    
    idx_origin_1 = Hash_Blk2Origin(HashTable_Idx, idx_blk_1);
    idx_origin_2 = Hash_Blk2Origin(HashTable_Idx, idx_blk_2);
    
    vec_all_1 = Hash_Blk2Alpha(HashTable_Idx, idx_blk_1);
    vec_all_2 = Hash_Blk2Alpha(HashTable_Idx, idx_blk_2);
    x1 = Alpha(vec_all_1(1));
    y1 = Alpha(vec_all_1(2));
    x2 = Alpha(vec_all_2(1));
    y2 = Alpha(vec_all_2(2));
    
    if idx_origin_1 <= 0 && idx_origin_2 <= 0
        plot([x1;x2],[y1;y2],'Color','k');
    end
end

% draw robot trajectory
idxvec_tmp = find(HashTable_Idx.idx_origin <= 0);
num_odo = numel(idxvec_tmp);
for i = 0:num_odo-1
    idx_org = -i;
    idx_tmp = find(HashTable_Idx.idx_origin == idx_org);
    idx_st_b_1 = HashTable_Idx.idx_st_b(idx_tmp);
    idx_tmp = find(HashTable_Idx.idx_origin == idx_org-1);
    idx_st_b_2 = HashTable_Idx.idx_st_b(idx_tmp);
    x1 = Alpha(idx_st_b_1);
    y1 = Alpha(idx_st_b_1+1);
    %     x2 = Alpha(idx_st_b_2);
    %     y2 = Alpha(idx_st_b_2+1);
    idx_submap = Hash_Org2Submap(HashTable_Dcmps, idx_org);
    colorIdx = mod(idx_submap,6);  
    markerSize = 10;
    switch colorIdx
        case 1
            colorCmd = 'r';
        case 2
            colorCmd = 'g';        
        case 0
            colorCmd = 'b';
        case 3
            colorCmd = 'c';
        case 4
            colorCmd = 'm';
        case 5
            colorCmd = 'y';
    end
    if idx_submap == 0
        colorCmd = 'k';
        markerSize = 20;
    end
    plot(x1,y1,'.','Color',colorCmd,'MarkerSize',markerSize);
    
    
end
% draw features
idxvec_tmp = find(HashTable_Idx.idx_origin > 0);
num_feature = numel(idxvec_tmp);
for i = 1:num_feature
    idx_tmp = idxvec_tmp(i);
    idx_b_tmp = HashTable_Idx.idx_st_b(idx_tmp);
    idx_org = HashTable_Idx.idx_origin(idx_tmp);
    x1 = Alpha(idx_b_tmp);
    y1 = Alpha(idx_b_tmp+1);
    idx_submap = Hash_Org2Submap(HashTable_Dcmps, idx_org);        
    colorIdx = mod(idx_submap,3);
    markerSize = 5;
    switch colorIdx
        case 1
            colorCmd = 'r';
        case 2
            colorCmd = 'g';
        case 0
            colorCmd = 'b';
    end
    if idx_submap == 0
        colorCmd = 'k';
        markerSize = 5;
    end
    plot(x1,y1,'*','Color',colorCmd,'MarkerSize',markerSize);
end


end

