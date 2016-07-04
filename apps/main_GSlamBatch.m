clear;

%% read local maps
disp('Loading local maps...');
[ HashTable_Idx, HashTable_Cnstr, Cnstr_Pool] = LoadLocalMaps( 'city10000', 0, 9999 );
disp('All local maps loaded!!!');

%% test
[ HashTable_Dcmps ] = Proc_DcmpsAuto( HashTable_Idx, HashTable_Cnstr );

%% figure init
fig1 = ViewInit();

%% node init
disp('Initing nodes state vector...');
Alpha = Proc_InitAlpha_2( HashTable_Cnstr, Cnstr_Pool, [0;0;0], [1] );
ViewSlamRes( fig1, Alpha, HashTable_Idx, HashTable_Cnstr );
disp('All nodes init done!!!')

%% main SLAM process
for j = 1:5    
    t1 = clock;
    [ Omega, Xi, Alpha ] = Proc_GraphSLAM( ...
        Alpha, HashTable_Idx, HashTable_Cnstr, Cnstr_Pool );
    t2 = clock;
    intv = etime(t2,t1);
    disp(['Loop: ', num2str(j), ' using ', num2str(intv), ' sec.']);    
%     ViewSlamRes( fig1, Alpha, HashTable_Idx, HashTable_Cnstr );
end

%% graph decompose
% [ HashTable_Dcmps ] = Proc_DcmpsSqr( Alpha, HashTable_Idx, HashTable_Cnstr);
% ViewSlamResHier( fig1, Alpha, HashTable_Idx, HashTable_Cnstr, HashTable_Dcmps );

[ HashTable_Dcmps, G_DcmpsN ] = Proc_DcmpsAuto( HashTable_Idx, HashTable_Cnstr );
fig1 = ViewInit();
ViewSlamResHier( fig1, Alpha, HashTable_Idx, HashTable_Cnstr, HashTable_Dcmps, G_DcmpsN );

