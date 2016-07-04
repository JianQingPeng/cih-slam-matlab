clear;

%% video init
VideoObj = VideoWriter('../graph_slam_city10000_2015_8_17_1.avi', 'Motion JPEG AVI' );
VideoObj.FrameRate = 30;
open(VideoObj);

%% plot init
fig = figure;
grid on;
hold on;
axis equal;
set(gca,'XTick',-1000:5:1000);
set(gca,'YTick',-1000:5:1000);
set(gca, 'xlim', [-60 60]);
set(gca, 'ylim', [-60 60]);
set(gcf,'position',[0,0,800,800]);

%% read local maps
NameDataSet = 'city10000';
switch NameDataSet
    case 'city10000'
        dataFold_path = '../data/city10000';
        num_localMaps = 9999;
        ReadStatus_IfPoseNegative = false;
    case 'Intel'
        dataFold_path = '../data/Intel';
        num_localMaps = 942;
        ReadStatus_IfPoseNegative = false;
    case 'manhattonOlson3500'
        dataFold_path = '../data/manhattanOlson3500';
        num_localMaps = 3499;
        ReadStatus_IfPoseNegative = false;
end
[ HashTable_Idx, HashTable_Cnstr, Cnstr_Pool ] = ...
    Proc_ReadData( dataFold_path, num_localMaps, ReadStatus_IfPoseNegative );
disp('Local maps loaded.');

%% main
Alpha_init = Proc_InitAlpha( HashTable_Idx, HashTable_Cnstr, Cnstr_Pool );

Alpha = sparse(3,1);
idx_LoopCloseLast = 0;

for i = 2:1:9999
    t1 = clock;    
    %% choose set of poses
    idx_blk_end = i;
    HashTable_Idx_now.idx_org = HashTable_Idx.idx_org(1:idx_blk_end);
    HashTable_Idx_now.idx_blk = HashTable_Idx.idx_blk(1:idx_blk_end);
    HashTable_Cnstr_now = HashTable_Cnstr(1:idx_blk_end,1:idx_blk_end);
    
    %% initialize the status vector alpha
    
    idx_cnstr = HashTable_Cnstr_now(end-1,end);
    e_tmp = Cnstr_Pool(idx_cnstr).e;
    vec_1 = Alpha(end-2:end);
    T_1_2 = Trans_Pose_to_Mat(e_tmp);
    T_w_1 = Trans_Pose_to_Mat(vec_1);
    T_w_2 = T_w_1*T_1_2;
    vec_2 = Trans_Mat_to_Pose(T_w_2);
    Alpha = [Alpha;sparse(vec_2)];
    
    num_cnstr_new = nnz(HashTable_Cnstr_now(:,end))+nnz(HashTable_Cnstr_now(end,:));
    LoopClosingDone = false;
    NumNotLoopClosing = 20;
    if num_cnstr_new > 1 & i-idx_LoopCloseLast > NumNotLoopClosing
        disp('Loop closing detected. Do global localization! ...')
        for j = 1:3
            %% main SLAM process
            [ Omega, Xi, Alpha ] = Proc_GraphSLAM( ...
                Alpha, HashTable_Idx_now, HashTable_Cnstr_now, Cnstr_Pool );
        end
        idx_LoopCloseLast = i;
        LoopClosingDone = true;
    elseif i-idx_LoopCloseLast > NumNotLoopClosing
        %         disp('Loop closing detected recently, refresh by odometry.')
    else
        %         disp('No loop closing detected. refresh by odometry.')
    end
    t2 = clock;
    
    Proc_DrawRes_seq( Alpha, HashTable_Idx_now, HashTable_Cnstr_now, LoopClosingDone);
    pause(0.01);
    
    frame = getframe(fig);
    writeVideo(VideoObj, frame);
    
    t3 = clock;
    intv_slam = etime(t2,t1);
    intv_draw = etime(t3,t2);
    disp(['Current LocalMap Index: ', num2str(i), ', slam time:', num2str(intv_slam), ', draw time:', num2str(intv_draw)]);
end

close(fig);
close(VideoObj);