clear all;

%% read local maps
[ HashTable_Idx, HashTable_Cnstr, Cnstr_Pool] = LoadLocalMaps( 'manhattonOlson3500' );

%% decompose graph
[ HashTable_Dcmps, G_DcmpsN ] = Proc_DcmpsAuto( HashTable_Idx, HashTable_Cnstr );

%% generate keynode graph
[ HashTable_Cnstr_Submap0, Cnstr_Pool ] = Proc_CihSlam_Extract( ...
    HashTable_Cnstr, HashTable_Dcmps, Cnstr_Pool );

%% solve graph in all levels
[ Alpha, Alpha_Submap0 ] = Proc_CihSlam_Solve( ...
    HashTable_Idx, HashTable_Dcmps, HashTable_Cnstr, HashTable_Cnstr_Submap0, Cnstr_Pool );

%% display results
fig1 = ViewInit();
ViewSlamRes( fig1, Alpha_Submap0, [], HashTable_Cnstr_Submap0, 'k', 15);
fig2 = ViewInit();
ViewSlamResHier( fig2, Alpha, [], HashTable_Cnstr, HashTable_Dcmps, G_DcmpsN );
