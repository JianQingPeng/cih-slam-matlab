function ViewSlamResHier( fig, Alpha, HashTable_Idx, HashTable_Cnstr, HashTable_Dcmps, G_DcmpsN )
%VIEWSLAMRESHIER Summary of this function goes here
%   Detailed explanation goes here
%% global refresh
figure(fig);

%% graph coloring
vec_color = GraphColoring( G_DcmpsN );

%% draw observation
[rows_tmp, cols_tmp] = find(HashTable_Cnstr);
num_cnstr = numel(rows_tmp);
for i = 1:num_cnstr
    idx_blk_1 = rows_tmp(i);
    idx_blk_2 = cols_tmp(i);
    
    vec_all_1 = 3*idx_blk_1-2:3*idx_blk_1;
    vec_all_2 = 3*idx_blk_2-2:3*idx_blk_2;
    x1 = Alpha(vec_all_1(1));
    y1 = Alpha(vec_all_1(2));
    x2 = Alpha(vec_all_2(1));
    y2 = Alpha(vec_all_2(2));
    
    plot([x1;x2],[y1;y2],'Color','b');
end

%% draw robot pose nodes
num_nodes = numel(Alpha)/3;
for i = 1:num_nodes
    idx_blk_tmp = i;
    idxvec_alpha_tmp = Hash_Blk2Alpha([], idx_blk_tmp);
    idx_submap = HashTable_Dcmps.idx_submap(idx_blk_tmp);
    
    vec_tmp = Alpha(idxvec_alpha_tmp);
    x1 = vec_tmp(1);
    y1 = vec_tmp(2);
    
    colorCmd = 'k';
    markerSizeCmd = 5;
    
    if idx_submap == 0
        colorCmd = 'k';
        markerSizeCmd = 15;
    else
        switch mod(vec_color(idx_submap),5)
            case 0
                colorCmd = 'b';
            case 1
                colorCmd = 'r';
            case 2
                colorCmd = 'g';
            case 3
                colorCmd = 'y';
            case 4
                colorCmd = 'm';
        end
    end
    
    plot(x1,y1,'.','Color',colorCmd,'MarkerSize',markerSizeCmd);
end

%% draw key nodes
idxvec_blk_Submap0 = find(HashTable_Dcmps.idx_submap == 0);
for i = 1:numel(idxvec_blk_Submap0)
    idx_blk_tmp = idxvec_blk_Submap0(i);
    idxvec_alpha_tmp = Hash_Blk2Alpha([],idx_blk_tmp);
    vec_tmp = Alpha(idxvec_alpha_tmp);
    x1 = vec_tmp(1);
    y1 = vec_tmp(2);
    plot(x1,y1,'.','Color','k','MarkerSize',15);
end

end

