function [ HashTable_Cnstr_Submap0, Cnstr_Pool ] = Proc_CihSlam_Extract( ...
    HashTable_Cnstr, HashTable_Dcmps, Cnstr_Pool )
%PROC_CIHSLAM_EXTRACT Summary of this function goes here
%   Detailed explanation goes here
idxvec_blk_Submap0 = find(HashTable_Dcmps.idx_submap == 0);
num_submap = max(HashTable_Dcmps.idx_submap);
num_submap0 = numel(idxvec_blk_Submap0);
HashTable_Cnstr_Submap0 = HashTable_Cnstr(idxvec_blk_Submap0, idxvec_blk_Submap0);

for idx_submap_now = 1:num_submap
    
    idxvec_blk_SubmapNow = find(HashTable_Dcmps.idx_submap == idx_submap_now);
    [rows1, cols1] = find(HashTable_Cnstr(idxvec_blk_SubmapNow, idxvec_blk_Submap0));
    [rows2, cols2] = find(HashTable_Cnstr(idxvec_blk_Submap0, idxvec_blk_SubmapNow));
    idxvec_blk_Submap0Now = unique([idxvec_blk_Submap0(cols1); idxvec_blk_Submap0(rows2)]);
    
    % Index vector of the combination map of keynodes related and the current
    % submap nodes.
    
    idxvec_blk_JointMapNow = [idxvec_blk_Submap0Now;idxvec_blk_SubmapNow];
    
    HashTable_Cnstr_JointMapNow = HashTable_Cnstr(idxvec_blk_JointMapNow,idxvec_blk_JointMapNow);
    [S, C] = graphconncomp(HashTable_Cnstr_JointMapNow + HashTable_Cnstr_JointMapNow.');
    
    for i = 1:S
        % generate hash table for the joint map now
        vec_tmp = find(C == i);
        idxvec_blk_JointMapNow_lv2 = idxvec_blk_JointMapNow(vec_tmp);
        num_JointMapNow_lv2 = numel(idxvec_blk_JointMapNow_lv2);
        
        idxvec_blk_Submap0Now_lv2 = Fun_FindKeyVec( idxvec_blk_JointMapNow_lv2, HashTable_Dcmps );
        num_Submap0Now_lv2 = numel(idxvec_blk_Submap0Now_lv2);
        
        HashTable_Cnstr_Submap0Now_lv2 = HashTable_Cnstr(idxvec_blk_JointMapNow_lv2,idxvec_blk_JointMapNow_lv2);
        
        % init alpha
        [ Alpha_JointMapNow_lv2 ] = Proc_InitAlpha_2( HashTable_Cnstr_Submap0Now_lv2, Cnstr_Pool, [0;0;0], [1] );
        
        % main loop for graph slam
        for SLAM_loop = (1:3)
            [ Omega_JointMapNow_lv2, Xi_JointMapNow_lv2, Alpha_JointMapNow_lv2 ] = Proc_GraphSLAM( ...
                Alpha_JointMapNow_lv2, [], HashTable_Cnstr_Submap0Now_lv2, Cnstr_Pool );
        end
        
        idxvec_Submap0Now_lv2 = (1:num_Submap0Now_lv2*3);
        Alpha_Submap0Now_lv2 = Alpha_JointMapNow_lv2(idxvec_Submap0Now_lv2);
        
        % draw result
        %     Proc_DrawRes_batch(Alpha_JointMapNow_lv2,[],HashTable_Cnstr_JointMapNow_lv2);
        
        % extract vector in rotation and translation
        idxvec_JointMapNow_rot_lv2 = (3:3:num_JointMapNow_lv2*3).';
        Alpha_JointMapNow_rot_lv2 = Alpha_JointMapNow_lv2(idxvec_JointMapNow_rot_lv2);
        idxvec_JointMapNow_tr_lv2 = sort([(1:3:num_JointMapNow_lv2*3-2).';(2:3:num_JointMapNow_lv2*3-1).']);
        Alpha_JointMapNow_tr_lv2 = Alpha_JointMapNow_lv2(idxvec_JointMapNow_tr_lv2);
        
        % condition: to generate pure rotation for Joint Map
        [ Omega_JointMapNow_rot_lv2, Xi_JointMapNow_rot_lv2 ] = Cal_InfoMatVec_Conditional( ...
            Omega_JointMapNow_lv2, Xi_JointMapNow_lv2, Alpha_JointMapNow_tr_lv2, idxvec_JointMapNow_tr_lv2 );
        
        % condition: to generate pure translation for Joint Map
        [ Omega_JointMapNow_tr_lv2, Xi_JointMapNow_tr_lv2 ] = Cal_InfoMatVec_Conditional( ...
            Omega_JointMapNow_lv2, Xi_JointMapNow_lv2, Alpha_JointMapNow_rot_lv2, idxvec_JointMapNow_rot_lv2 );
        
        % marginal: to generate pure rotation for Submap 0
        idxvec_tmp = (1:num_Submap0Now_lv2);
        [ Omega_Submap0Now_rot_lv2, Xi_Submap0Now_rot_lv2 ] = Cal_InfoMatVec_Marginal( ...
            Omega_JointMapNow_rot_lv2, Xi_JointMapNow_rot_lv2, idxvec_tmp );
        
        % marginal: to generate pure tranlation for submap 0
        idxvec_tmp = (1:num_Submap0Now_lv2*2);
        [ Omega_Submap0Now_tr_lv2, Xi_Submap0Now_tr_lv2 ] = Cal_InfoMatVec_Marginal( ...
            Omega_JointMapNow_tr_lv2, Xi_JointMapNow_tr_lv2, idxvec_tmp );
        
        % generate constraint between key nodes
        HashTable_Weight_Submap0Now_lv2 = sparse(num_Submap0Now_lv2,num_Submap0Now_lv2);
        HashTable_Cnstr_Submap0Now_lv2 = sparse(num_Submap0Now_lv2,num_Submap0Now_lv2);
        for i2 = 1:num_Submap0Now_lv2-1
            for j2 = i2+1:num_Submap0Now_lv2
                [ I_tmp, e_tmp ] = Fun_GenCnstr( ...
                    Omega_Submap0Now_rot_lv2, Omega_Submap0Now_tr_lv2, Alpha_Submap0Now_lv2, i2, j2 );
                Cnstr_tmp.I = I_tmp;
                Cnstr_tmp.e = e_tmp;
                Cnstr_Pool{end+1} = Cnstr_tmp;
                HashTable_Cnstr_Submap0Now_lv2(i2,j2) = numel(Cnstr_Pool);
                
                if det(I_tmp) ~= 0
                    Weight_now = 1/det(I_tmp);
                else
                    Weight_now = Inf;
                end
                HashTable_Weight_Submap0Now_lv2(i2,j2) = Weight_now;
            end
        end
        
        % simplify graph by shortest path and large loop remaining
        G = HashTable_Weight_Submap0Now_lv2+HashTable_Weight_Submap0Now_lv2.';
        [Tree, Pred] = graphminspantree(G);
        [rows, cols] = find(G < 10*max(max(Tree)));
%         [rows2, cols2] = find(Tree);
%         for i5 = 1:numel(rows2)
%             idx1 = rows2(i5);
%             idx2 = cols2(i5);
%             x1 = Alpha_Submap0Now_lv2(3*idx1-2);
%             y1 = Alpha_Submap0Now_lv2(3*idx1-1);
%             x2 = Alpha_Submap0Now_lv2(3*idx2-2);
%             y2 = Alpha_Submap0Now_lv2(3*idx2-1);
%             dist_tmp = norm([x1-x2; y1-y2]);
%             Tree(idx1,idx2) = dist_tmp;
%             Tree(idx2,idx1) = dist_tmp;
%         end
        Tree = boolean(Tree+Tree.');
        diameter_Tree = max(max(graphallshortestpaths(Tree)));
        for i4 = 1:numel(rows)
            dist = graphshortestpath(Tree, rows(i4), cols(i4));
            if dist > 0.3*diameter_Tree
                idx1 = rows(i4);
                idx2 = cols(i4);
                x1 = Alpha_Submap0Now_lv2(3*idx1-2);
                y1 = Alpha_Submap0Now_lv2(3*idx1-1);
                x2 = Alpha_Submap0Now_lv2(3*idx2-2);
                y2 = Alpha_Submap0Now_lv2(3*idx2-1);
                dist_tmp = norm([x1-x2; y1-y2]);
                Tree(idx1,idx2) = dist_tmp;
                Tree(idx2,idx1) = dist_tmp;
            end
        end
        Tree = boolean(Tree+Tree.');
        HashTable_Cnstr_Submap0Now_lv2 = HashTable_Cnstr_Submap0Now_lv2.*Tree;
        
        % refresh hash table constraint for key nodes
        vec_tmp = HashTable_Dcmps.idx_local_blk(idxvec_blk_Submap0Now_lv2);
        
        [rows, cols] = find(HashTable_Cnstr_Submap0Now_lv2);
        for i3 = 1:numel(rows)
            i_tmp = vec_tmp(rows(i3));
            j_tmp = vec_tmp(cols(i3));
            
            if HashTable_Cnstr_Submap0(i_tmp,j_tmp) == 0
                HashTable_Cnstr_Submap0(i_tmp,j_tmp) = HashTable_Cnstr_Submap0Now_lv2(rows(i3),cols(i3));
            else
                idx_cnstr_1 = HashTable_Cnstr_Submap0(i_tmp,j_tmp);
                idx_cnstr_2 = HashTable_Cnstr_Submap0Now_lv2(rows(i3),cols(i3));
                I1 = Cnstr_Pool{idx_cnstr_1}.I;
                e1 = Cnstr_Pool{idx_cnstr_1}.e;
                I2 = Cnstr_Pool{idx_cnstr_2}.I;
                e2 = Cnstr_Pool{idx_cnstr_2}.e;
                [I3, e3] = Fun_Merge_InfoMatVec( I1, e1, I2, e2 );
                Cnstr_tmp.I = I3;
                Cnstr_tmp.e = e3;
                Cnstr_Pool{end+1} = Cnstr_tmp;
                HashTable_Cnstr_Submap0(i_tmp,j_tmp) = numel(Cnstr_Pool);
            end
        end
        HashTable_Cnstr_Submap0_all(vec_tmp,vec_tmp) = ones(numel(vec_tmp)) - eye(numel(vec_tmp));
    end
    
    disp(['Submap: ', num2str(idx_submap_now), ' extracted.']);
end

end

