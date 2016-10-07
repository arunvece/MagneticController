function hu_1_area(mapNum,alg)

%Collects particles that can not overlap using {l,r,u,d} moves in a 2D
%  bounded map. The moves are implimented on all particles and so there is global control.
% Black represents boundaries and obstacles and red
% represents the particles.
% Pick two robots, A and B.
% pA = position (A), pB = position(B)
% use BFS and APSP to find moveSeq that moves robot A to pB
% apply moveSeq to all the robots.
% if robots A reaches goal B, return to step 2
% if all robots overlap (are at one position) end
% else, return to step 1.
%  There are 3 algorithms that can be implemented
%  using this program.
%  9- Collect the largest swarms

% There are 21 maps to choose from- 1 through 17, 500,501,502,6000

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global G
format compact
G.i=1;
G.fig = figure(1);
G.numCommands = 0;
G.totalMoves = 0;
roicrob=[0,1];
%% Check input. If more than 1 argument, then take input as assigned, else use map 500 and algorithm 9
if nargin >1
    G.mapNum = mapNum;
    G.alg = alg;
else
    G.mapNum = 500;
    G.alg = 9;
end
%% Setup the map
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Massive Control','color','w')
set(gca,'FontSize',20);


[G.obstacle_pos,G.free,G.robvec,G.Moves,G.ri,G.ci,G.apspD] = SetupWorld();
G.sizeSteps = 1000;
G.uniquePos = ones(G.sizeSteps,1);
G.step = 0;
G.movecount = 0;
clf
G.colormap = [  1,1,1; %Empty = white
    0,0,0; %obstacle
    1,0,0; %robot
    %      0,1,0;%roi
    ];

im = G.obstacle_pos;
freesize=size(G.robvec);
mult=rand(freesize);
G.robvec(mult>0.3)=0;
im(G.free) = 2*G.robvec;
G.no_ros=size(im(im==2),1);
if numel(unique(im))<3  %trick so that the right colors show up even with all cells filled
    colormap(G.colormap(2:end,:));
else
    colormap(G.colormap(1:3,:));
end
G.axis=imagesc(im);
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
axis equal; axis tight
updateTitle()
im2=zeros(size(im));
im2(im==1)=0;
im2(im==2)=1;
im2=im2bw(im2);
%             im2=flip(im2);
%             figure(2)
%              imshow(im2);
G.s2=bwconncomp(im2,4);

hold on
%% The main loop
tic  %start a timer
G.pairwisePath = [];
cRobVec = G.robvec;
SizeRoi=10;
if G.alg==9
    [G.rgn,G.xi,G.yi]=roipoly;
end
% run the algorithm until fullr roi marked in blue, is covered:
while SizeRoi>0
    
    roicrob=[0,1];
    overlap=G.rgn.*im2;
    roioccup=find(overlap);
    for q=1:size(roioccup,1)
        roicrob(q)=find(G.free==roioccup(q,1));
        cRobVec(roicrob(q))=0;
    end
    [rgnpts1,rgnpts2]=find(G.rgn);
    %             scatter(rgnpts2,rgnpts1);
    G.sizeroi=size(rgnpts1);
    G.rgn(roioccup)=0;
    roileft=find(G.rgn);
    roioccup=find(overlap);
    
    [cRobVec, pathSeg] =  pairwiseCombine(cRobVec);
    G.pairwisePath  = [G.pairwisePath,pathSeg];
    
    hold on
    SizeRoi=size(roileft,1);
    a(:,1)=G.ri;
    a(:,2)=G.ci;
    %      scatter(G.ci(G.A),G.ri(G.A),'*');
    %      scatter(G.ci(G.B),G.ri(G.B));
    drawnow
end
shortpath = G.pairwisePath;
display('KEY: l,r,u,d = 1,2,3,4')
display(['(',num2str(length(shortpath)),' steps): ',num2str(shortpath)])
totalTime = toc;

% save the number of steps and the seconds
saveName = 'data/timeAndPathLength.mat';
load(saveName)
dataSec(G.alg, G.mapNum) = totalTime; %#ok<NASGU>
% dataPathLength(G.alg, G.mapNum) = numel(shortpath); %#ok<NASGU>
dataFreeSpace(G.mapNum) = numel(G.free); %#ok<NASGU>
datauniquePos{G.alg,G.mapNum} = G.uniquePos; %#ok<NASGU>
save(saveName, 'dataSec','dataPathLength','dataFreeSpace','datauniquePos');

display(['Elapsed time is ',num2str(totalTime),' seconds.'])
figure(1)
%% Area calculations
    function AreaCalc()
        s = regionprops(G.s2,'centroid','Area');
        A       = [s.Area];
        biggest = find(A == max(A));
        biggest=biggest(1);
        %         imshow(G.s2);
        %          figure(2)
        %          imshow(im2)
        centroid(:,:)=s(biggest).Centroid;
        G.pacent=centroid;
        %         scatter(G.pacent(1),G.pacent(2));
    end
%% Move A to position of B until A abd B overlap
    function [cRobVec, pathSeg] = pairwiseCombine(cRobVec)
        % Pick two robots, A and B. This is governed by the algorithm
        % number passed to it.
        AreaCalc();
        [pA,pB] = choosepApB(cRobVec, G.apspD);
        pathSeg = [];
        %  if robots A and B do not overlap try again
        AreaCalc();
        %               pB = find(VecB == 1,1,'first');
        
        hold on
        %               [px,py]=ind2sub(size(im),pA);
        %              scatter(G.ri(pA),G.ci(pA),'*');
        %              scatter(G.ri(pB),G.ci(pB));
        drawnow
        % 3. use BFS to find moveSeq that moves robot A to pB
        map_dist = BFSdistmap(G.obstacle_pos, G.ri(pB),G.ci(pB)); % to see the mapdist, call image(map_dist,'CDataMapping','scaled')
        moveSeq  =  BFSshortestRoute(map_dist,G.ri(pA),G.ci(pA));
        % 4. apply moveSeq to all the robots.
        steps = min(inf,numel(moveSeq));
        for mvIn =1:steps
            cRobVec = applyMove(moveSeq(mvIn), cRobVec);
            G.s2=bwconncomp(im2,4);
            moveto(moveSeq(mvIn),cRobVec);
            
            overlap=G.rgn.*im2;
            roioccup=find(overlap);
            for q=1:size(roioccup,1)
                roicrob(q)=find(G.free==roioccup(q,1));
                cRobVec(roicrob(q))=0;
            end
            
            G.rgn(roioccup)=0;
            roileft=find(G.rgn);
            roioccup=find(overlap);
            SizeRoi= size(roileft,1);
            G.s2=bwconncomp(im2,4);
            G.step = G.step +1;
            if G.step > G.sizeSteps %grow the matrix
                G.uniquePos = [G.uniquePos;ones(1000,1)];
                G.sizeSteps = numel(G.uniquePos);
                hold on
            end
            G.uniquePos(G.step) = sum(cRobVec);
            
        end
        pathSeg = [pathSeg, moveSeq(1:steps)]; %#ok<AGROW>
        % 5. calculate the new positions of robot A and robot B.
        %             VecA = zeros(size(cRobVec)); VecA(pA) = 1;
        %             VecB = zeros(size(cRobVec)); VecB(pB) = 1;
        % %             for mvIn =1:steps
        %                 VecA = applyMove(moveSeq(mvIn), VecA);
        %                 VecB = applyMove(moveSeq(mvIn), VecB);
        %             end
        Irob = find(im ~=1);
        AreaCalc();
        
        
        G.A=pA;
        G.B=pB;
        
        
    end
%% % returns an array giving the shortest distanct from [r_start,
% c_start] to the location with distance 0 on the map map_dist
%display('KEY: l,r,u,d = 1,2,3,4')
    function moveSeq  = BFSshortestRoute(map_dist,r_start, c_start)
        
        r = r_start;
        c = c_start;
        d = map_dist(r, c);
        plen = d;
        moveSeq = zeros(1,map_dist(r, c));
        rOffsets = [0,0,-1,1]; %move l,r,u,d
        cOffsets = [-1,1,0,0]; %move l,r,u,d
        for i = 1:plen
            d = map_dist(r, c);
            for j = 1:4
                if map_dist(r+rOffsets(j), c+cOffsets(j)) == d - 1;
                    moveSeq(i) = j;
                    r = r+rOffsets(j); c = c+cOffsets(j);
                    break;
                end
            end
        end
    end
%% determines BFS distance on map_obst  from freespace at [r_start, c_start] to
%all other freespaces. Assumes map_obst is a 2D grid map of ones
%and zeros. Zeros represent freespaces.

    function map_dist = BFSdistmap(map_obst, r_start, c_start)
        numfree = sum(map_obst(:) == 0);
        map_dist = -2+int32(map_obst); %-1 is obstacle, -2 is unexplored
        map_dist(r_start, c_start) = 0;
        leafNodes = zeros(numfree,2);
        leafptr = 1;
        leafNodes(leafptr,:)  = [r_start, c_start];
        rOffsets = [0,0,1,-1]; %move l,r,u,d
        cOffsets = [-1,1,0,0]; %move l,r,u,d
        for ct = 1:numfree
            rl = leafNodes(ct,1);
            cl = leafNodes(ct,2);
            d = map_dist(rl,cl);
            for dir = 1:4
                ro = rl+rOffsets(dir); co = cl+cOffsets(dir);
                if map_dist(ro,co) == -2
                    map_dist(ro,co) = d+1; %put shortest distance on map
                    leafptr = leafptr+1; %add this to the leaf pointers
                    leafNodes(leafptr,:)  = [ro,co];
                    if leafptr >= numfree
                        break;
                    end
                end
            end
            if leafptr >= numfree
                break;
            end
        end  %end for loop
    end
%% 1. Pick two robots, A and B by passing the current robot position vector and the distance matrix
    function [pA,pB] = choosepApB(cRobVec,apspD)
        
        if G.alg == 5
            [pA,pB] = twoCloseNodes(cRobVec,apspD); %5 = greedy with closestNodes
        elseif G.alg == 6
            [pA,pB] = twoFurthestNodes(cRobVec,apspD);
            pA;%6 = greedy with furthest
        elseif G.alg == 7      %7 = Greedy with first two particles
            Irob = find(cRobVec == 1,2,'first');
            pA = Irob(1);pB = Irob(2);
        elseif G.alg ==8
            Irob = find(cRobVec == 1);
            randInds = randperm(numel(Irob));
            pA = Irob(randInds(1));pB = Irob(randInds(2));
        elseif G.alg==9
            Irob = find(im ~=1);
            G.s2=bwlabel(im2,4);
            
            pA=nearest([G.pacent(1),G.pacent(2)]);
            [rgnpts1,rgnpts2]=find(G.rgn);
            pB =[rgnpts2(1),rgnpts1(1)];
            
            pB=sub2ind(size(im),pB(2), pB(1));
            tmp=abs(Irob-pB);
            [val, idx]=min(tmp);
            pB=idx;
            pA=sub2ind(size(im),pA(2), pA(1));
            tmp = abs(Irob-pA);
            
            [val,idx] = min(tmp); %index of closest value
            %  closest = Irob(idx); %closest value
            pA= idx;
            %             pB=sub2ind(size(im),pB(1),pB(2));
            %             q=pB;
        else
            Irob = find(cRobVec == 1,2,'first');
            pA = Irob(1);pB = Irob(2);
            %pB = find(cRobVec == 1,1,'last'); G.alg = 6;%
        end
    end
%% Returns the linear indexed position of the position of pair A,B having shortest distance in
% world map.
    function [pA,pB] = twoCloseNodes(cRobVec, apspD)
        %pick the first robot
        curMin = Inf;
        pA = find(cRobVec>0,1,'first');
        pB = find(cRobVec>0,1,'last');
        %1.) select the columns of apspD that are non zero in cRobVec
        for i = 1:numel(cRobVec)
            if cRobVec(i) > 0
                % 2.) in each column, select the rows that are nonzero in cRobVec
                dists = apspD(:,i);
                dists(dists==0)=inf;
                dists(cRobVec == 0) = Inf;
                [tmin,tind] = min(dists);
                %  3.) select the minimum value if it is larger than current max
                if tmin<curMin
                    pA = i; pB = tind;
                end
            end
        end
    end
%% Returns the linear indexed position of the position of pair A,B having largest distance
% in world map.
    function [pA,pB] = twoFurthestNodes(cRobVec, apspD)
        curMax = 0;
        pA = find(cRobVec>0,1,'first');
        pB = find(cRobVec>0,1,'last');
        %1.) select the columns of apspD that are non zero in cRobVec
        for i = 1:numel(cRobVec)
            if cRobVec(i) >0
                % 2.) in each column, select the rows that are nonzero in cRobVec
                dists = apspD(:,i).*cRobVec;
                [tmax,tind] = max(dists);
                %  3.) select the maximum value if it is larger than current max
                if tmax>curMax
                    pA = i; pB = tind;
                end
            end
        end
    end
%% Implement the move on every robot
    function rvec2 = applyMove(mv, rvecIn)
        rvec2 = zeros(size(rvecIn));
        if mv==1 || mv==3
            for ni = 1:numel(rvecIn)
                if rvecIn(G.Moves(ni,mv)) ~= 1
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;
                else
                    rvec2(ni)=rvecIn(ni);
                end
                rvecIn(ni)=rvec2(ni);
            end
        else
            for ni=numel(rvecIn):-1:1
                if rvecIn(G.Moves(ni,mv)) == 0
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;
                else
                    rvec2(ni)=rvecIn(ni);
                end
            end
            rvecIn(ni)=rvec2(ni);
            
        end
        
    end
%% Can be called to interactively solve the puzzle manually
    function keyhandler(src,evnt) %#ok<INUSL>
        if strcmp(evnt.Key,'s')
            imwrite(flipud(get(G.axis,'CData')+1), G.colormap, '../../pictures/png/MatrixPermutePic.png');
        else
            %             moveto(evnt.Key)
        end
    end
%% Maps keypress to moving pixels
    function moveto(key,crobvec)
        
        if strcmp(key,'r')  %RESET
            [G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld();
            return
        end
        if strcmp(key,'p') %save a picture
            set(gcf,'PaperPositionMode','auto','PaperSize',[1.5,1.9], 'PaperPosition',[0,0,1.5,1.9] );
            print(gcf, '-dpdf', ['test',num2str(G.movecount,'%05d'),'.pdf']);
            return
        end
        mv=0;
        if strcmp(key,'leftarrow') || strcmp(key,'a')|| key ==1 %-x
            mv = 1;
        elseif strcmp(key,'rightarrow')|| strcmp(key,'d')|| key ==2   %+x
            mv = 2;
        elseif strcmp(key,'uparrow')|| strcmp(key,'w')|| key==3  %+y
            mv = 3;
        elseif strcmp(key,'downarrow')|| strcmp(key,'x') || key==4 %-y
            mv = 4;
        end
        if mv>0
            G.movecount = G.movecount+1;
            %             G.robvec = applyMove(mv, G.robvec);
            im = G.obstacle_pos;
            im(G.free) = 2*crobvec;
            %             im(G.rgn)=4;
            if numel(unique(im))<2
                colormap(G.colormap(2:end,:));
            else
                colormap(G.colormap);
            end
            clf(1)
            G.axis=imagesc(im);
            set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
            axis equal; axis tight
            updateTitle()
            hold on
            %             G.rgn=flip(G.rgn);
            G.rgn(G.obstacle_pos==1)=0;
            [rgnpts1,rgnpts2]=find(G.rgn);
            scatter(rgnpts2,rgnpts1);
            G.sizeroi=size(rgnpts1);
            
            %              scatter(G.ri(G.A),G.ci(G.A));
            %            scatter(G.ri(G.B),G.ci(G.B));
            drawnow
            im2=zeros(size(im));
            im2(im==1)=0;
            im2(im==2)=1;
            im2=im2bw(im2);
            %             figure(3)
            %             imshow(im2);
            %                          im2=flip(im2);
            
            %             figure(2)
            %              imshow(im2);
            %              drawnow();
            G.s2=bwlabel(im2,4);
            
            
            hold on
            %             plot(G.centroids(:,1),G.centroids(:,2), 'b*')
            
            hold off
        end
    end
    function updateTitle()
        title({[num2str(G.movecount), ' moves'];[num2str(sum(G.robvec)),' unique particles']})
    end
%%      % Blk is the position of obstacles
% free is the index of the free spaces in blk
% robvec is a binary vector where the ith element is true if there
% is a robot at free(i).
    function [blk,free,robvec,Moves,ri,ci,apspD] = SetupWorld()
        
        fn = ['data/SetupWorld', num2str(G.mapNum,'%03d'),'.mat'];
        if exist(fn, 'file') == 2
            load(fn)
            G.mapsize=size(blk);
            return;
        end
        tic
        display(['Generating data for map ',num2str(G.mapNum,'%03d')])
        
        if G.mapNum == 502
            blk = imread('leafBW.png');
            blk= im2bw(blk);
            
        elseif G.mapNum == 500
            blk = imread('leafBWsmall.png');
            blk= im2bw(blk);
            
        elseif G.mapNum == 501
            blk = imread('leafBWmedium.png');
            blk= im2bw(blk);
        elseif G.mapNum==6000
            blk = imread('leafBWbig.png');
            blk= im2bw(blk);
        else
            blk=~blockMaps(G.mapNum); Blockmaps is a function
        end
        
        
        m2=size(blk,1);
        n2=size(blk,2);
        map_inputz=zeros(m2,n2);
        map_inputz(blk == 1) = 0;
        map_inputz(blk == 0) = 1;
        blk=map_inputz;
        free = find(blk==0);
        robvec = ones(size(free));
        imshow(blk);
        [ri,ci] = find(blk==0);
        AdjcM = sparse(numel(robvec),numel(robvec));
        G.mapsize=size(blk);
        %Moves(i,3) gives the index in robvec after applying an up move.
        Moves = repmat( (1:numel(free))',1,4);
        world = -blk;
        world(free) = 1:numel(free);
        % hardcode mapping: if I move up, what does that map to in world?
        % I could do this as a vector multiplication
        %l,r,u,d
        %TODO only save lower matrix  This results in the upper triangle of the sparse matrix being ignored.
        for i = 1:numel(free)
            r = ri(i);
            c = ci(i);
            if blk(r,c-1) == 0
                indx = world(r,c-1);
                Moves(i,1) = indx; %left
                if i ~= indx
                    AdjcM(max(i,indx),min(i,indx)) = 1; %#ok<*SPRIX>
                end
            end
            if blk(r,c+1) == 0
                indx = world(r,c+1); %right
                Moves(i,2) = indx;
                if i ~= indx
                    AdjcM(max(i,indx),min(i,indx)) = 1;
                end
            end
            if blk(r-1,c) == 0
                indx = world(r-1,c); %down
                Moves(i,3) = indx;
                if i ~= indx
                    AdjcM(max(i,indx),min(i,indx)) = 1;
                end
            end
            if blk(r+1,c) == 0
                indx =  world(r+1,c);%up
                Moves(i,4) = indx;
                if i ~= indx
                    AdjcM(max(i,indx),min(i,indx)) = 1;
                end
            end
        end
        % calculate shortest paths
        apspD = graphallshortestpaths(AdjcM,'Directed', false);
        %Johnson's algorithm has a time complexity of O(N*log(N)+N*E), where NF and E are the number of nodes and edges respectively.
        % [1] Johnson, D.B. (1977). Efficient algorithms for shortest paths in sparse networks. Journal of the ACM 24(1), 1-13.
        %[2] Siek, J.G., Lee, L-Q, and Lumsdaine, A. (2002). The Boost Graph Library User Guide and Reference Manual, (Upper Saddle River, NJ:Pearson Education).
        
        
        save(fn,'blk','free','robvec','Moves','ri','ci','apspD');
        toc
    end
end