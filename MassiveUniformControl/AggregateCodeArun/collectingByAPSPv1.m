function collectingByAPSPv1(mapNum,alg)

%  collectingByAPSPv1(mapNum,alg):
%Collects particles that can overlap using {l,r,u,d} moves in a 2D
%  bounded map. The moves are implimented on all particles and so there is global control. 
% Black represents boundaries and obstacles and red
% represents the particles. 
% Pick two robots, A and B.
% pA = position (A), pB = position(B)
% use BFS and APSP to find moveSeq that moves robot A to pB
% apply moveSeq to all the robots.
% if robots A and B do not overlap return to step 2
% if all robots overlap (are at one position) end
% else, return to step 1.
%  There are 6 algorithms that can be implemented
%  using this program.
%  5- Collect closest pair of particles
%  6- Collect farthest pair of particles
%  7- Collect the first two pairs of particles seen while scanning from top
%  left to right bottom.
%  8 through 17- Random pairs are collected
%  20- Collect first particle to the last particle while scanning from top
%  left to right bottom
% 21- Collect first particle to middle particle while scanning from top
% left to right bottom
% There are 21 maps to choose from- 1 through 17, 500,501,502,6000

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pauseTs = 0.01;
clc
format compact
global G
MOVIE_NAME = 'collectAPSP1'; %to make a movie 'Friction_bots.mp4'
G.fig = figure(1);
clf
writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
set(writerObj,'Quality',100);
writerObj.FrameRate=30;
open(writerObj);
    function makemymovie()% each frame has to be added. So a function is made and called whenever an image needs to be added
        figure(G.fig)
        F = getframe;
        writeVideo(writerObj,F.cdata);
      %  writeVideo(writerObj,F.cdata);
    end
G.fig = figure(1);
G.numCommands = 0;
G.totalMoves = 0;
%% Check input. If more than 1 argument, then take input as assigned, else use map 500 and algorithm 6   
if nargin >1
    G.mapNum = mapNum;
    G.alg = alg;
else
    G.mapNum = 500;
    G.alg = 6;
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
    ];
im = G.obstacle_pos;
im(G.free) = 2*G.robvec;
if numel(unique(im))<3  %trick so that the right colors show up even with all cells filled
    colormap(G.colormap(2:end,:));
else
    colormap(G.colormap);
end
G.axis=imagesc(im);
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
axis equal; axis tight
updateTitle()
hold on
%% The main loop 
tic  %start a timer
G.pairwisePath = [];
cRobVec = G.robvec;

% run the algorithm until only one particle is left:
while sum(cRobVec)>1
    [cRobVec, pathSeg] =  pairwiseCombine(cRobVec);
    G.pairwisePath  = [G.pairwisePath,pathSeg];
end
 shortpath = G.pairwisePath;  
display('KEY: l,r,u,d = 1,2,3,4')
display(['(',num2str(length(shortpath)),' steps): ',num2str(shortpath)])
totalTime = toc;

% save the number of steps and the seconds
saveName = 'data/timeAndPathLength.mat';
load(saveName)
dataSec(G.alg, G.mapNum) = totalTime; %#ok<NASGU>
dataPathLength(G.alg, G.mapNum) = numel(shortpath); %#ok<NASGU>
dataFreeSpace(G.mapNum) = numel(G.free); %#ok<NASGU>
datauniquePos{G.alg,G.mapNum} = G.uniquePos; %#ok<NASGU>
save(saveName, 'dataSec','dataPathLength','dataFreeSpace','datauniquePos');

display(['Elapsed time is ',num2str(totalTime),' seconds.'])

figure(1)
%% Move A to position of B until A abd B overlap
    function [cRobVec, pathSeg] = pairwiseCombine(cRobVec)
        % Pick two robots, A and B. This is governed by the algorithm
        % number passed to it.
        [pA,pB] = choosepApB(cRobVec, G.apspD);
        pathSeg = [];
        while pA ~= pB %  if robots A and B do not overlap try again
            % 3. use BFS to find moveSeq that moves robot A to pB
            map_dist = BFSdistmap(G.obstacle_pos, G.ri(pA),G.ci(pA)); % to see the mapdist, call image(map_dist,'CDataMapping','scaled')
            moveSeq  =  BFSshortestRoute(map_dist,G.ri(pB),G.ci(pB));
            % 4. apply moveSeq to all the robots.
            steps = min(inf,numel(moveSeq));
            for mvIn =1:steps
                cRobVec = applyMove(moveSeq(mvIn), cRobVec);
                G.step = G.step +1;
                if G.step > G.sizeSteps %grow the matrix
                    G.uniquePos = [G.uniquePos;ones(1000,1)];
                    G.sizeSteps = numel(G.uniquePos);
                end
                G.uniquePos(G.step) = sum(cRobVec);
                
            end
            pathSeg = [pathSeg, moveSeq(1:steps)]; %#ok<AGROW>
            % 5. calculate the new positions of robot A and robot B.
            VecA = zeros(size(cRobVec)); VecA(pA) = 1;
            VecB = zeros(size(cRobVec)); VecB(pB) = 1;
            for mvIn =1:steps
                VecA = applyMove(moveSeq(mvIn), VecA);
                VecB = applyMove(moveSeq(mvIn), VecB);
            end
            pA = find(VecA == 1,1,'first');
            pB = find(VecB == 1,1,'first');
        end
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
            [pA,pB] = twoFurthestNodes(cRobVec,apspD); %6 = greedy with furthest
         elseif G.alg == 7      %7 = Greedy with first two particles 
            Irob = find(cRobVec == 1,2,'first');
            pA = Irob(1);pB = Irob(2);
        elseif G.alg > 7 && G.alg <18
            Irob = find(cRobVec == 1);
            randInds = randperm(numel(Irob));
            pA = Irob(randInds(1));pB = Irob(randInds(2));
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
        for ni = 1:numel(rvecIn)
            if rvecIn(ni)
                rvec2(G.Moves(ni,mv)) = rvecIn(ni);
            end
        end
    end

%% Can be called to interactively solve the puzzle manually  
    function keyhandler(src,evnt) %#ok<INUSL>
        if strcmp(evnt.Key,'s')
            imwrite(flipud(get(G.axis,'CData')+1), G.colormap, '../../pictures/png/MatrixPermutePic.png');
        else
            moveto(evnt.Key)
        end
    end
%% Maps keypress to moving pixels
    function moveto(key)
        
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
        if strcmp(key,'leftarrow') || strcmp(key,'a')|| strcmp(key,'1') %-x
            mv = 1;
        elseif strcmp(key,'rightarrow')|| strcmp(key,'d')|| strcmp(key,'2')  %+x
            mv = 2;
        elseif strcmp(key,'uparrow')|| strcmp(key,'w')|| strcmp(key,'3')  %+y
            mv = 4;
        elseif strcmp(key,'downarrow')|| strcmp(key,'x') || strcmp(key,'4') %-y
            mv = 3;
        end
        if mv>0
            G.movecount = G.movecount+1;
%             G.robvec = applyMove(mv, G.robvec);
            im = G.obstacle_pos;
            im(G.free) = 2*crobvec;
            if numel(unique(im))<3
                colormap(G.colormap(2:end,:));
            else
                colormap(G.colormap);
            end
            clf(1)
            G.axis=imagesc(im);
            set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
            axis equal; axis tight
            updateTitle()
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
            blk=~blockMaps(G.mapNum); %Blockmaps is a function
        end
        
        
        m2=size(blk,1);
        n2=size(blk,2);
        map_inputz=zeros(m2,n2);
        map_inputz(blk == 1) = 0;
        map_inputz(blk == 0) = 1;
        blk=map_inputz;
        free = find(blk==0);
        robvec = ones(size(free));
        [ri,ci] = find(blk==0);
        AdjcM = sparse(numel(robvec),numel(robvec));
        
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