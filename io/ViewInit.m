function [ fig ] = ViewInit()
%VIEWINIT Summary of this function goes here
%   Detailed explanation goes here
fig = figure;
hold on;
grid off;
axis equal;
% set(gca,'XTick',-1000:5:1000);
% set(gca,'YTick',-1000:5:1000);
set(gca, 'xlim', [-50 45]);
% set(gca, 'ylim', [-23 5]);
% set(gcf,'position',[0,0,800,800]);
set(gca,'position',[0.05 0.05 0.9 0.9])

end

