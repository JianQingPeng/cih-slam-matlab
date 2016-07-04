function [ vec_color ] = GraphColoring( G )
%GRAPHCOLORING Summary of this function goes here
%   Detailed explanation goes here

num = length(G);
vec_color = -ones(num,1);

for i = 1:num    
    vec1 = find(G(i,:)).';
    vec2 = find(G(:,i));
    vec = [vec1;vec2];    
    for j = 1:99
        tmp = find(vec_color(vec) == j, 1);
        if isempty(tmp)
            vec_color(i) = j;
            break;
        end
    end
end

tmp = find(vec_color == -1, 1);
if ~isempty(tmp)
    error('Error in graph coloring!!!')
end


end

