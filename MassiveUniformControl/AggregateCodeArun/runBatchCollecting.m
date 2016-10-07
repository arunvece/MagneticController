%%TODO: plot unique_locations as function of number of moves for 3 algs
fs = 20;


%%  run collectingByAPSPv1(mapNum,alg)

for maps = 1:22
    for algs = 5:7
        collectingByAPSPv1(maps,algs)
    end
end

% %%  run collectingByAPSPv1(mapNum,alg)
% 
% for maps = 500:502
%     for algs = 5:7
%         collectingByAPSPv1(maps,algs)
%     end
% end

% %%  run collectingByAPSPv1(mapNum,alg)
% 
% for maps = 500
%     for algs = 8:17
%         collectingByAPSPv1(maps,algs)
%     end
% end



%%
load('data/timeAndPathLength.mat')
%dataSec(G.alg, G.mapNum) = totalTime; %#ok<NASGU>
%dataPathLength(G.alg, G.mapNum) = numel(shortpath); %#ok<NASGU>
% dataFreeSpace(G.mapNum) = numel(G.free);

[Ys,Is ] = sort(dataFreeSpace);

nonObs = [14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30];
pathL =  [10, 11, 12, 11, 9, 9, 14, 15, 15, 16, 15, 18, 16, 17, 17, 18, 19];


figure(3);set(gcf,'color','w');
plot( dataFreeSpace(Is), dataSec( 5,Is),...
    dataFreeSpace(Is), dataSec( 6,Is),...
    dataFreeSpace(Is), dataSec( 7,Is))
xlabel('Non obstacle spaces')
ylabel('computation time (s)')
legend('closest particles','furthest particles','connect to first')

figure(4);set(gcf,'color','w');

%TODO: add data for 2000, 4000, 6000 freespaces

plot( dataFreeSpace(Is), dataPathLength( 5,Is),...
    dataFreeSpace(Is), dataPathLength( 6,Is),...
    dataFreeSpace(Is), dataPathLength( 7,Is))
xlabel('Non obstacle spaces')
ylabel('Moves required to converge')
legend('closest particles','furthest particles','connect to first')

%%

% [10,  9, 15,  18,  16, 17, 17, 18, 19]/dataPathLength( 5,inds) gives 0.54
% as the 

inds = Is(end-23:end-7);dataFreeSpace(inds)

figure(7); clf; set(gcf,'color','w');set(gca,'FontSize',fs)
plot( dataFreeSpace(inds), dataPathLength( 5,inds),'s-',...
    dataFreeSpace(inds), dataPathLength( 6,inds),'o-',...
    dataFreeSpace(inds), dataPathLength( 7,inds),'d-',...
nonObs,pathL,'^-', 'linewidth',1.5,'markersize',10 )
xlabel('Non obstacle spaces')
ylabel('Moves to converg')
axis([13.5,30.5,0,75])
legend('closest particles','furthest particles','connect to first','optimal','location','best')

set(gcf,'PaperPositionMode','auto','PaperSize',[8,4], 'PaperPosition',[0,0,8,4] );
print(gcf, '-dpdf', 'OptimalVsGreedy.pdf');

dataPathLength( 5,inds)/pathL
%%
load('data/timeAndPathLength.mat')
figure(5); clf; set(gcf,'color','w');set(gca,'FontSize',fs)
semilogy(1:numel(datauniquePos{5,500}), datauniquePos{5,500},...
    1:numel(datauniquePos{6,500}), datauniquePos{6,500},...
    1:numel(datauniquePos{7,500}), datauniquePos{7,500}, 'linewidth',1.5    )
hold on
for i = 8:17
    semilogy(1:numel(datauniquePos{i,500}), datauniquePos{i,500},'m-'   )
    
end
semilogy(1:numel(datauniquePos{5,500}), datauniquePos{5,500},...
    1:numel(datauniquePos{6,500}), datauniquePos{6,500},...
    1:numel(datauniquePos{7,500}), datauniquePos{7,500}, 'linewidth',1.5    )
xlabel('Number of moves')
ylabel('Unique robot positions')
legend('closest particles','furthest particles','connect to first','random A,B')

set(gcf,'PaperPositionMode','auto','PaperSize',[8,4], 'PaperPosition',[0,0,8,4] );
print(gcf, '-dpdf', 'SmallLeafUniquePositions.pdf');
