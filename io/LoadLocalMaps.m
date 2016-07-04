function [ HashTable_Idx, HashTable_Cnstr, Cnstr_Pool] = LoadLocalMaps( ...
    NameDataSet, startIdx_org, endIdx_org )
%LOADLOCALMAPS: load local maps from raw data    

%% set path of local maps
switch NameDataSet
    case 'city10000'
        dataFold_path = 'data/city10000';
        num_localMaps = 9999;
    case 'Intel'
        dataFold_path = 'data/Intel';
        num_localMaps = 942;
    case 'manhattonOlson3500'
        dataFold_path = 'data/manhattanOlson3500';
        num_localMaps = 3499;
end

if nargin < 3
    startIdx_org = 0;
    endIdx_org = num_localMaps;
end
num_localMapsLoaded = endIdx_org-startIdx_org+1;

%% read local maps
HashTable_Idx = struct('idx_org', [], 'idx_blk', []);
HashTable_Idx.idx_org = (startIdx_org:endIdx_org).';
HashTable_Idx.idx_blk = (1:num_localMapsLoaded).';
HashTable_Cnstr = sparse(num_localMapsLoaded,num_localMapsLoaded);
Cnstr_Element_tmp = struct('I',[],'e',[]);
Cnstr_Pool = cell(0);
for i = 1:num_localMaps
    dataFile_path = [dataFold_path, '/localmap_', int2str(i), '.mat' ];
    load(dataFile_path);
    if Ref < startIdx_org || Ref > endIdx_org
        continue;
    end
    Idx_blk_1_tmp = Ref - startIdx_org +1;
    num_LocalMapLinked = numel(st)/2/3;
    for j = 1:num_LocalMapLinked
        if st(j*3-2,1) < startIdx_org || st(j*3-2,1) > endIdx_org
            continue;
        end
        Idx_blk_2_tmp = st(j*3-2,1)- startIdx_org +1;
        if (HashTable_Cnstr(Idx_blk_1_tmp,Idx_blk_2_tmp) == 0)
            HashTable_Cnstr(Idx_blk_1_tmp,Idx_blk_2_tmp) = numel(Cnstr_Pool)+1;
            I_tmp = I(j*3-2:j*3, j*3-2:j*3);
            e_tmp = st(j*3-2:j*3, 2);
            Cnstr_Element_tmp.I = I_tmp;
            Cnstr_Element_tmp.e = e_tmp;
            Cnstr_Pool{end+1} = Cnstr_Element_tmp;
        end
    end
end

end

