function Proc_DrawRes_seq( Alpha, HashTable_Idx, HashTable_Cnstr, LoopClosingDone)
%PROC_DRAWRES_V2 Summary of this function goes here
%   Detailed explanation goes here
% figure;

%% global refresh
if LoopClosingDone
    hold off
    plot(1,1);
    hold on;
    grid on;
    axis equal;
    set(gca,'XTick',-1000:5:1000);
    set(gca,'YTick',-1000:5:1000);
    set(gca, 'xlim', [-60 60]);
    set(gca, 'ylim', [-60 60]);
    set(gcf,'position',[0,0,800,800]);
    
    %% draw observation relations
    [rows_tmp, cols_tmp] = find(HashTable_Cnstr);
    num_cnstr = numel(rows_tmp);
    for i = 1:num_cnstr
        idx_blk_1 = rows_tmp(i);
        idx_blk_2 = cols_tmp(i);
        
        idx_org_1 = Hash_Blk2Org(HashTable_Idx, idx_blk_1);
        idx_org_2 = Hash_Blk2Org(HashTable_Idx, idx_blk_2);
        
        vec_all_1 = 3*idx_blk_1-2:3*idx_blk_1;
        vec_all_2 = 3*idx_blk_2-2:3*idx_blk_2;
        x1 = Alpha(vec_all_1(1));
        y1 = Alpha(vec_all_1(2));
        x2 = Alpha(vec_all_2(1));
        y2 = Alpha(vec_all_2(2));
        
        if idx_org_1 <= 0 && idx_org_2 <= 0
            plot([x1;x2],[y1;y2],'Color','k');
        end
    end
    
    %% draw robot trajectory
    idxvec_tmp = find(HashTable_Idx.idx_org <= 0);
    num_odo = numel(idxvec_tmp);
    for i = 1:num_odo
        idx_tmp = idxvec_tmp(i);
        idx_st_b_1 = idx_tmp*3-2;
        x1 = Alpha(idx_st_b_1);
        y1 = Alpha(idx_st_b_1+1);
        plot(x1,y1,'.','Color','k','MarkerSize',5);
    end
    
    %% draw features
    idxvec_tmp = find(HashTable_Idx.idx_org > 0);
    num_feature = numel(idxvec_tmp);
    for i = 1:num_feature
        idx_tmp = idxvec_tmp(i);
        idx_b_tmp = HashTable_Idx.idx_st_b(idx_tmp);
        idx_org = HashTable_Idx.idx_org(idx_tmp);
        x1 = Alpha(idx_b_tmp);
        y1 = Alpha(idx_b_tmp+1);
        idx_submap = Hash_Org2Submap(HashTable_Dcmps, idx_org);
        markerSize = 5;
        colorCmd = 'r';
        plot(x1,y1,'*','Color',colorCmd,'MarkerSize',markerSize);
    end
    
else
    %% incremental draw
    
    x1 = Alpha(end-5);
    y1 = Alpha(end-4);
    x2 = Alpha(end-2);
    y2 = Alpha(end-1);
    %     plot([x1;x2],[y1;y2],'Color','b');
    plot(x1,y1,'.','Color','b','MarkerSize',5);
    plot(x2,y2,'.','Color','r','MarkerSize',5);
    
    vec_tmp = find(HashTable_Cnstr(end,:));
    for i = 1:numel(vec_tmp)
        idx_blk_1 = vec_tmp(i);
        x1 = Alpha(idx_blk_1*3-2);
        y1 = Alpha(idx_blk_1*3-1);
        plot([x1;x2],[y1;y2],'Color','b');
    end
    vec_tmp = find(HashTable_Cnstr(:,end));
    for i = 1:numel(vec_tmp)
        idx_blk_1 = vec_tmp(i);
        x1 = Alpha(idx_blk_1*3-2);
        y1 = Alpha(idx_blk_1*3-1);
        plot([x1;x2],[y1;y2],'Color','b');
    end
    
    
end

end

