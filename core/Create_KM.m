function [ HashTable_Cnstr_SM0, Cnstr_Pool_Cell ] = Create_KM( JMInfo, Cnstr_Pool_Cell )
%CREATE_KM Summary of this function goes here
%   generate key map in constraint form from a joint map

HashTable_Cnstr_JM = JMInfo.HashTable_Cnstr;
idxvec_blk_JM = JMInfo.idxvec_blk_JM;
idxvec_blk_key = JMInfo.idxvec_blk_key;
idxvec_blk_norm = JMInfo.idxvec_blk_norm;
[S, C] = graphconncomp(HashTable_Cnstr_JM + HashTable_Cnstr_JM.');

num_key = numel(idxvec_blk_key);
HashTable_Cnstr_SM0 = sparse(num_key,num_key);

for i = 1:S
    % generate hash table for the joint map now
    idxvec_local_lv2 = find(C == i);
    idxvec_blk_JM_lv2 = idxvec_blk_JM(idxvec_local_lv2);
    num_JM_lv2 = numel(idxvec_blk_JM_lv2);
    HashTable_Cnstr_JM_lv2 = HashTable_Cnstr_JM(idxvec_local_lv2,idxvec_local_lv2);
    
    idxvec_local_SM0_lv2 = find(C(1:num_key) == i);
    idxvec_blk_SM0_lv2 = idxvec_blk_key(idxvec_local_SM0_lv2);
    num_SM0_lv2 = numel(idxvec_blk_SM0_lv2);
    
    % init alpha
    [ Alpha_JM_lv2 ] = Proc_InitAlpha_2( HashTable_Cnstr_JM_lv2, Cnstr_Pool_Cell );
    
    % main loop for graph slam
    for SLAM_loop = (1:3)
        [ Omega_JM_lv2, Xi_JM_lv2, Alpha_JM_lv2 ] = Proc_GraphSLAM( ...
            Alpha_JM_lv2, [], HashTable_Cnstr_JM_lv2, Cnstr_Pool_Cell );
    end
    
    idxvec_SM0_lv2 = (1:num_SM0_lv2*3);
    Alpha_SM0_lv2 = Alpha_JM_lv2(idxvec_SM0_lv2);
    
    % extract vector in rotation and translation
    idxvec_JM_rot_lv2 = (3:3:num_JM_lv2*3).';
    Alpha_JM_rot_lv2 = Alpha_JM_lv2(idxvec_JM_rot_lv2);
    idxvec_JM_tr_lv2 = sort([(1:3:num_JM_lv2*3-2).';(2:3:num_JM_lv2*3-1).']);
    Alpha_JM_tr_lv2 = Alpha_JM_lv2(idxvec_JM_tr_lv2);
    
    % condition: to generate pure rotation for Joint Map
    [ Omega_JM_rot_lv2, Xi_JM_rot_lv2 ] = Cal_InfoMatVec_Conditional( ...
        Omega_JM_lv2, Xi_JM_lv2, Alpha_JM_tr_lv2, idxvec_JM_tr_lv2 );
    
    % condition: to generate pure translation for Joint Map
    [ Omega_JM_tr_lv2, Xi_JM_tr_lv2 ] = Cal_InfoMatVec_Conditional( ...
        Omega_JM_lv2, Xi_JM_lv2, Alpha_JM_rot_lv2, idxvec_JM_rot_lv2 );
    
    % marginal: to generate pure rotation for Submap 0
    idxvec_tmp = (1:num_SM0_lv2);
    [ Omega_SM0_rot_lv2, Xi_SM0_rot_lv2 ] = Cal_InfoMatVec_Marginal( ...
        Omega_JM_rot_lv2, Xi_JM_rot_lv2, idxvec_tmp );
    
    % marginal: to generate pure tranlation for submap 0
    idxvec_tmp = (1:num_SM0_lv2*2);
    [ Omega_SM0_tr_lv2, Xi_SM0_tr_lv2 ] = Cal_InfoMatVec_Marginal( ...
        Omega_JM_tr_lv2, Xi_JM_tr_lv2, idxvec_tmp );
    
    % generate constraint between key nodes
    HashTable_Weight_SM0_lv2 = sparse(num_SM0_lv2,num_SM0_lv2);
    HashTable_Cnstr_SM0_lv2 = sparse(num_SM0_lv2,num_SM0_lv2);
    for i2 = 1:num_SM0_lv2-1
        for j2 = i2+1:num_SM0_lv2
            [ I_tmp, e_tmp ] = Fun_GenCnstr( ...
                Omega_SM0_rot_lv2, Omega_SM0_tr_lv2, Alpha_SM0_lv2, i2, j2 );
            Cnstr_tmp.I = I_tmp;
            Cnstr_tmp.e = e_tmp;
            
            num_cnstr = numel(Cnstr_Pool_Cell);
            Cnstr_Pool_Cell{num_cnstr+1} = Cnstr_tmp;
            HashTable_Cnstr_SM0_lv2(i2,j2) = numel(Cnstr_Pool_Cell);
            
            if det(I_tmp) ~= 0
                Weight_now = 1/det(I_tmp);
            else
                Weight_now = Inf;
            end
            HashTable_Weight_SM0_lv2(i2,j2) = Weight_now;
        end
    end
    [Tree, Pred] = graphminspantree(HashTable_Weight_SM0_lv2+HashTable_Weight_SM0_lv2.');
    Tree = boolean(Tree+Tree.');
    HashTable_Cnstr_SM0_lv2 = HashTable_Cnstr_SM0_lv2.*Tree;
    
    % bio-direction constriant
    [rows, cols] = find(HashTable_Cnstr_SM0_lv2);
    for i4 = 1:numel(rows)
        idx_cnstr_tmp = HashTable_Cnstr_SM0_lv2(rows(i4), cols(i4));
        Cnstr_tmp = Cnstr_Pool_Cell{idx_cnstr_tmp};
        T_tmp = Trans_Pose_to_Mat(Cnstr_tmp.e);
        inv_T_tmp = inv(T_tmp);
        Cnstr_tmp.e = Trans_Mat_to_Pose(inv_T_tmp);
        Cnstr_Pool_Cell{end+1} = Cnstr_tmp;
        HashTable_Cnstr_SM0_lv2(cols(i4),rows(i4)) = numel(Cnstr_Pool_Cell);
    end
    
    % refresh hash table constraint for key nodes
    [rows, cols] = find(HashTable_Cnstr_SM0_lv2);
    idxvec_local_lv2 = find(C(1:num_key) == i);
    
    for i3 = 1:numel(rows)
        i_local = idxvec_local_lv2(rows(i3));
        j_local = idxvec_local_lv2(cols(i3));
        
        if HashTable_Cnstr_SM0(i_local,j_local) == 0
            HashTable_Cnstr_SM0(i_local,j_local) = HashTable_Cnstr_SM0_lv2(rows(i3),cols(i3));
        else
            idx_cnstr_1 = HashTable_Cnstr_SM0(i_local,j_local);
            idx_cnstr_2 = HashTable_Cnstr_SM0_lv2(rows(i3),cols(i3));
            I1 = Cnstr_Pool_Cell{idx_cnstr_1}.I;
            e1 = Cnstr_Pool_Cell{idx_cnstr_1}.e;
            I2 = Cnstr_Pool_Cell{idx_cnstr_2}.I;
            e2 = Cnstr_Pool_Cell{idx_cnstr_2}.e;
            [I3, e3] = Fun_Merge_InfoMatVec( I1, e1, I2, e2 );
            Cnstr_tmp.I = I3;
            Cnstr_tmp.e = e3;
            Cnstr_Pool_Cell{end+1} = Cnstr_tmp;
            HashTable_Cnstr_SM0(i_local,j_local) = numel(Cnstr_Pool_Cell);
        end
    end
end



end

