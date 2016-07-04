function [ I, e ] = Fun_GenCnstr( Omega_rot, Omega_tr, Alpha, idx_blk_1, idx_blk_2 )
%FUN_GENCNSTR Summary of this function goes here
%   generate constraint between two nodes idx_1 and idx_2,
%   from the total information matrix.

idxvec_1 = (idx_blk_1*3-2: idx_blk_1*3).';
idxvec_2 = (idx_blk_2*3-2: idx_blk_2*3).';

Alpha_1 = Alpha(idxvec_1);
Alpha_2 = Alpha(idxvec_2);

e = zeros(3,1);
e(3) = Alpha_2(3) - Alpha_1(3);
e(3) = Trans_Period(e(3),pi,-pi);
theta_1 = Alpha_1(3);
R_1 = [cos(theta_1) -sin(theta_1); sin(theta_1) cos(theta_1)];
e(1:2) = R_1.'*(Alpha_2(1:2) - Alpha_1(1:2));

I = zeros(3,3);
I(3,3) = -Omega_rot(idx_blk_1,idx_blk_2);
Omega_tr_now = Omega_tr(idx_blk_1*2-1:idx_blk_1*2, idx_blk_2*2-1:idx_blk_2*2);

I(1:2,1:2) = -R_1.'*Omega_tr_now*R_1;
I(1:2,1:2) = (I(1:2,1:2)+I(1:2,1:2).')/2;

end

