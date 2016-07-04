function [ Alpha, Alpha_Submap0 ] = Proc_CihSlam_Solve( ...
    HashTable_Idx, HashTable_Dcmps, HashTable_Cnstr, HashTable_Cnstr_Submap0, Cnstr_Pool )
%PROC_CIHSLAM_SOLVE Summary of this function goes here
%   Detailed explanation goes here
%% solve submap 0
% init alpha
[ Alpha_Submap0 ] = Proc_InitAlpha_2( HashTable_Cnstr_Submap0, Cnstr_Pool, [0;0;0], 1 );

% main loop for graph slam
for SLAM_loop = (1:6)
    [ Omega_Submap0, Xi_Submap0, Alpha_Submap0 ] = Proc_GraphSLAM( ...
        Alpha_Submap0, [], HashTable_Cnstr_Submap0, Cnstr_Pool );
end
disp('Submap: 0 solved');

%% solve normal submaps

num_all = numel(HashTable_Idx.idx_blk);
Alpha = sparse(num_all,1);

idxvec_blk_Submap0 = find(HashTable_Dcmps.idx_submap == 0);
idxvec_alpha_Submap0 = Hash_Blk2Alpha( HashTable_Idx, idxvec_blk_Submap0 );
Alpha(idxvec_alpha_Submap0) = Alpha_Submap0;

num_submap = max(HashTable_Dcmps.idx_submap);
num_submap0 = numel(idxvec_blk_Submap0);

for idx_submap_now = 1:num_submap

    idxvec_blk_SubmapNow = find(HashTable_Dcmps.idx_submap == idx_submap_now);
    if numel(idxvec_blk_SubmapNow) == 0
        continue;
    end

    [rows1, cols1] = find(HashTable_Cnstr(idxvec_blk_SubmapNow, idxvec_blk_Submap0));
    [rows2, cols2] = find(HashTable_Cnstr(idxvec_blk_Submap0, idxvec_blk_SubmapNow));
    idxvec_blk_Submap0Now = unique([idxvec_blk_Submap0(cols1); idxvec_blk_Submap0(rows2)]);

    % Index vector of the combination map of keynodes related and the current
    % submap nodes.

    idxvec_blk_JointMapNow = [idxvec_blk_Submap0Now;idxvec_blk_SubmapNow];
    Alpha_JointMapNow = sparse(3*numel(idxvec_blk_JointMapNow));
    HashTable_Cnstr_JointMapNow = HashTable_Cnstr(idxvec_blk_JointMapNow,idxvec_blk_JointMapNow);

    vec_tmp = HashTable_Dcmps.idx_local_blk(idxvec_blk_Submap0Now);
    vec_tmp = sort([vec_tmp*3-2;vec_tmp*3-1;vec_tmp*3]);
    Alpha_Submap0Now = Alpha_Submap0(vec_tmp);
    idxvec_blk_tmp = (1:numel(Alpha_Submap0Now)/3).';

    % init alpha
    [ Alpha_JointMapNow ] = Proc_InitAlpha_2( ...
        HashTable_Cnstr_JointMapNow, Cnstr_Pool, Alpha_Submap0Now, idxvec_blk_tmp);

    for i2 = 1:3
        % generate joint map
        [ Omega_JointMapNow, Xi_JointMapNow] = Cal_InfoMatVec_Batch(  ...
            Alpha_JointMapNow, HashTable_Cnstr_JointMapNow, Cnstr_Pool );

        % condition: solve normal nodes
        num_tmp = numel(Alpha_Submap0Now);
        [ Omega_JointMapNow_rot_lv2, Xi_JointMapNow_rot_lv2 ] = Cal_InfoMatVec_Conditional( ...
            Omega_JointMapNow, Xi_JointMapNow, Alpha_Submap0Now, (1:num_tmp).');
        Alpha_MapNow = Omega_JointMapNow_rot_lv2\Xi_JointMapNow_rot_lv2;
        Alpha_JointMapNow = [Alpha_Submap0Now;Alpha_MapNow];
    end

    idxvec_alpha_SubmapNow = Hash_Blk2Alpha( HashTable_Idx, idxvec_blk_SubmapNow );
    Alpha(idxvec_alpha_SubmapNow) = Alpha_MapNow;
    
    disp(['Submap: ', num2str(idx_submap_now), ' solved.']);
end

end

