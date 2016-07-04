clear;

%% read local maps
disp('Loading local maps...');
NameDataSet = 'city10000';
switch NameDataSet
    case 'city10000'
        dataFold_path = 'data/city10000';
        idx_last_localMaps = 9999;
        ReadStatus_IfPoseNegative = false;
    case 'Intel'
        dataFold_path = 'data/Intel';
        idx_last_localMaps = 942;
        ReadStatus_IfPoseNegative = false;
    case 'manhattonOlson3500'
        dataFold_path = 'data/manhattanOlson3500';
        idx_last_localMaps = 3499;
        ReadStatus_IfPoseNegative = false;
end
[ HashTable_Idx, HashTable_Cnstr, Cnstr_Pool ] = ...
    Proc_ReadData( dataFold_path, idx_last_localMaps, ReadStatus_IfPoseNegative );
disp('Local maps loaded.');

%% read graph decompose information
disp('Loading graph decompose information...');
load('data/city10000_DcmpsInfo.mat');
disp('Graph decompose information loaded.');

%% video init
VideoObj = VideoWriter('cih_slam_city10000.avi', 'Motion JPEG AVI' );
VideoObj.FrameRate = 10;
open(VideoObj);

%% plot init
fig = figure;
grid off;
hold on;
axis equal;
set(gca,'XTick',-1000:5:1000);
set(gca,'YTick',-1000:5:1000);
set(gca, 'xlim', [-70 70]);
set(gca, 'ylim', [-70 70]);
set(gcf,'position',[0,0,800,800]);

%% change constraint pool into cell format
Cnstr_Pool_Cell = cell(1,1);
for i = 1:numel(Cnstr_Pool)
    Cnstr_Pool_Cell{i} = Cnstr_Pool(i);
end

%% main loop
num_submap = max(HashTable_Dcmps.idx_submap);
JMInfo_Pool = cell(num_submap,1);

JMInfo_SM0.HashTable_Cnstr = sparse(0,0);
JMInfo_SM0.idxvec_blk_JM = 1;
JMInfo_SM0.idxvec_blk_key = 1;
JMInfo_SM0.idxvec_blk_norm = [];

Alpha_SM0 = [0;0;0];

for idx_blk_now = 1:10000
    
    JMInfo_tmp.HashTable_Cnstr = [];
    JMInfo_tmp.idxvec_blk_JM = [];
    JMInfo_tmp.idxvec_blk_key = [];
    JMInfo_tmp.idxvec_blk_norm = [];
    JMInfo_tmp.alpha = [];
    
    idx_submap = HashTable_Dcmps.idx_submap(idx_blk_now);
    if idx_submap ~= 0
        % normal node now
        % find index vector for joint map now
        [ idxvec_blk_JM, idxvec_blk_key, idxvec_blk_norm ] = Create_JM_idxvec( ...
            HashTable_Dcmps, HashTable_Cnstr, idx_blk_now, idx_submap );
        JMInfo_tmp.idxvec_blk_JM = idxvec_blk_JM;
        JMInfo_tmp.idxvec_blk_key = idxvec_blk_key;
        JMInfo_tmp.idxvec_blk_norm = idxvec_blk_norm;
        % create constraints for joint map now
        [ HashTable_Cnstr_JM ] = Create_JM_cnstr( idxvec_blk_JM, HashTable_Cnstr);
        JMInfo_tmp.HashTable_Cnstr = HashTable_Cnstr_JM;
    else
        % key node now
        idxvec_blk_SM0 = find(HashTable_Dcmps.idx_submap(1:idx_blk_now) == 0);
        JMInfo_SM0.HashTable_Cnstr = blkdiag(JMInfo_SM0.HashTable_Cnstr,0);
        JMInfo_SM0.idxvec_blk_JM = idxvec_blk_SM0;
        JMInfo_SM0.idxvec_blk_key = idxvec_blk_SM0;
        JMInfo_SM0.idxvec_blk_norm = [];
        
        % find all submap linked to current node
        [ idxvec_submap ] = Find_Submap_Linked( ...
            idx_blk_now, idx_blk_now, HashTable_Cnstr, HashTable_Dcmps );
        % refresh joint map and submap 0
        for i = 1:numel(idxvec_submap)
            idx_submap = idxvec_submap(i);
            if idx_submap == 0
                % refresh submap 0 for single map;
                [ HashTable_Cnstr_SM0 ] = Create_JM_cnstr( idxvec_blk_SM0, HashTable_Cnstr);
                JMInfo_SM0.HashTable_Cnstr = HashTable_Cnstr_SM0;
                
            else
                % refresh joint map linked to current key node
                [ idxvec_blk_JM, idxvec_blk_key, idxvec_blk_norm ] = Create_JM_idxvec( ...
                    HashTable_Dcmps, HashTable_Cnstr, idx_blk_now, idx_submap );
                JMInfo_tmp.idxvec_blk_JM = idxvec_blk_JM;
                JMInfo_tmp.idxvec_blk_key = idxvec_blk_key;
                JMInfo_tmp.idxvec_blk_norm = idxvec_blk_norm;
                % create constraints for joint map now
                [ HashTable_Cnstr_JM ] = Create_JM_cnstr( idxvec_blk_JM, HashTable_Cnstr);
                JMInfo_tmp.HashTable_Cnstr = HashTable_Cnstr_JM;
                JMInfo_Pool{idx_submap} = JMInfo_tmp;
                
            end
        end
        
        % refresh submap 0 from joint map related
        if ~isempty(idxvec_submap)
            idxvec_submap_norm = idxvec_submap;
            if idxvec_submap_norm(1) == 0
                idxvec_submap_norm(1) = [];
            end
            for i = 1:numel(idxvec_submap_norm)
                idx_submap = idxvec_submap_norm(i);
                JMInfo = JMInfo_Pool{idx_submap};
                [ HashTable_Cnstr_SM0_JM, Cnstr_Pool_Cell ] = Create_KM( JMInfo, Cnstr_Pool_Cell );
                JMInfo_Pool{idx_submap}.HashTable_Cnstr_SM0 = HashTable_Cnstr_SM0_JM;
            end
        end
        
        % create submap 0 in upper level from all JM and SM0
        [ HashTable_Cnstr_SM0_ALL, Cnstr_Pool_Cell ] = Create_KM_cnstr_combined( ...
            JMInfo_SM0, JMInfo_Pool, HashTable_Dcmps, Cnstr_Pool_Cell);
        if nnz(HashTable_Cnstr_SM0_ALL) ~= 0
            % solve submap 0
            [ Alpha_SM0 ] = Proc_InitAlpha_2( HashTable_Cnstr_SM0_ALL, Cnstr_Pool_Cell, Alpha_SM0);
            for slam_loop = 1:3
                [ Omega_SM0, Xi_SM0, Alpha_SM0 ] = Proc_GraphSLAM( ...
                    Alpha_SM0, [], HashTable_Cnstr_SM0_ALL, Cnstr_Pool_Cell );
            end
        end
        
        % solve last submap conditioned on key nodes
        if (idx_blk_now > 1)
            idx_submap_last = HashTable_Dcmps.idx_submap(idx_blk_now-1);
            
            if (idx_submap_last > 0)
                idxvec_blk_key = JMInfo_Pool{idx_submap_last}.idxvec_blk_key;
                idxvec_local_blk_SM0now = HashTable_Dcmps.idx_local_blk(idxvec_blk_key);
                idxvec_tmp = Hash_Blk2Alpha([],idxvec_local_blk_SM0now);
                Alpha_SM0_now = Alpha_SM0(idxvec_tmp);
                HashTable_Cnstr_JM = JMInfo_Pool{idx_submap_last}.HashTable_Cnstr;
                [ Alpha_JM_now ] = Proc_InitAlpha_2( ...
                    HashTable_Cnstr_JM, Cnstr_Pool_Cell, Alpha_SM0_now );
                for i2 = 1:3
                    % generate joint map
                    [ Omega_JM, Xi_JM] = Cal_InfoMatVec_Batch(  ...
                        Alpha_JM_now, HashTable_Cnstr_JM, Cnstr_Pool_Cell );
                    % condition: solve normal nodes
                    num_tmp = numel(Alpha_SM0_now);
                    [ Omega_SM_now, Xi_SM_now ] = Cal_InfoMatVec_Conditional( ...
                        Omega_JM, Xi_JM, Alpha_SM0_now, (1:num_tmp).');
                    Alpha_SM_now = Omega_SM_now\Xi_SM_now;
                    Alpha_JM_now = [Alpha_SM0_now;Alpha_SM_now];
                end
                JMInfo_tmp = JMInfo_Pool{idx_submap_last};
                JMInfo_tmp.Alpha_JM = Alpha_JM_now;
                JMInfo_tmp.Alpha_SM = Alpha_SM_now;
                % renew joint map information
                JMInfo_Pool{idx_submap_last} = JMInfo_tmp;
            end
        end
        
        % draw result
        hold off;
        plot(0,0);
        Proc_DrawRes_batch( Alpha_SM0, [], HashTable_Cnstr_SM0_ALL, 'k', 15);
        if (idx_blk_now > 1)
            idx_submap_last = HashTable_Dcmps.idx_submap(idx_blk_now-1);
            if (idx_submap_last > 0)
                
                Alpha_JM_last = JMInfo_Pool{idx_submap_last}.Alpha_JM;
                HashTable_Cnstr_JM_last = JMInfo_Pool{idx_submap_last}.HashTable_Cnstr;
                Proc_DrawRes_batch( Alpha_JM_last, [], HashTable_Cnstr_JM_last, 'r', 5);
            end
        end
        frame = getframe(fig);
        writeVideo(VideoObj, frame);
        
    end
    
    %% debug
    disp(num2str(idx_blk_now));
end

close(VideoObj);
Proc_DrawRes_batch( Alpha_SM0, [], HashTable_Cnstr_SM0_ALL);
