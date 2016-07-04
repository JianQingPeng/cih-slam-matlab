function Proc_DrawRes_batch( Alpha, HashTable_Idx, HashTable_Cnstr, color_cmd, size_cmd)
%PROC_DRAWRES_V2 Summary of this function goes here
%   Detailed explanation goes here

if nargin ~= 5
    color_cmd = 'b';
    size_cmd = 5;
end

%% global refresh
% figure;

hold off;
plot(0,0);

hold on;
grid off;
axis equal;
set(gca,'XTick',-1000:5:1000);
set(gca,'YTick',-1000:5:1000);
set(gca, 'xlim', [-70 70]);
set(gca, 'ylim', [-70 70]);
set(gcf,'position',[0,0,800,800]);


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
    plot([x1;x2],[y1;y2],'Color',color_cmd);

end

%% draw robot pose nodes
num_nodes = numel(Alpha)/3;
for i = 1:num_nodes
    x1 = Alpha(3*i-2);
    y1 = Alpha(3*i-1);
    plot(x1,y1,'.','Color',color_cmd,'MarkerSize',size_cmd);
end
pause(0.1);
end

