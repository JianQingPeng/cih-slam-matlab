function [ Omega, Xi] = Cal_InfoMatVec_Batch(  ...
    Alpha, HashTable_Cnstr, Cnstr_Pool_Cell )
%CAL_INFOMATVEC_BATCH Summary of this function goes here
%   Detailed explanation goes here

    num_all = numel(Alpha);
    Omega = spalloc(num_all, num_all, num_all*20);
    Xi = spalloc(num_all, 1, num_all);

    [rows, cols] = find(HashTable_Cnstr);
    num_cnstr = numel(rows);
    
    for i = 1:num_cnstr
        idx_cnstr = HashTable_Cnstr(rows(i), cols(i));
        I_tmp = Cnstr_Pool_Cell{idx_cnstr}.I;
        e_tmp = Cnstr_Pool_Cell{idx_cnstr}.e;
        
        vec_alpha_1_tmp = (rows(i)*3-2:rows(i)*3);
        vec_alpha_2_tmp = (cols(i)*3-2:cols(i)*3);
                
        p1_tmp = Alpha(vec_alpha_1_tmp);
        p2_tmp = Alpha(vec_alpha_2_tmp);
        
        [ Omega_tmp, Xi_tmp ] = Cal_InfoMatVec(p1_tmp, p2_tmp, I_tmp, e_tmp );
%         Omega_tmp_sp = sparse(Omega_tmp);
%         Xi_tmp_sp = sparse(Xi_tmp);
        vec_alpha_12_tmp = [vec_alpha_1_tmp, vec_alpha_2_tmp];
        Omega(vec_alpha_12_tmp,vec_alpha_12_tmp) = Omega(vec_alpha_12_tmp,vec_alpha_12_tmp) + sparse(Omega_tmp);
        Xi(vec_alpha_12_tmp) = Xi(vec_alpha_12_tmp) + sparse(Xi_tmp);
    end
    
    Omega(1:3,1:3) = Omega(1:3,1:3);
end

