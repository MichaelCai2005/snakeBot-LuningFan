%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input parameters into code
%  1. KK -- wave parameter (close to number of segments)
%  2. AA -- angular amplitude**
%  3. ww -- temporal frequency
%  4. sp -- inner peg spacing**
%  5. wpx -- peg x position**
%  6. stopTime -- how long in seconds to run simulation
%  7. saveVideo -- 1 for yes, 0 for no
%  8. RFTstartTime -- when (in seconds) to turn RFT on (need to find best time to minimize initial z-translation.)
%  9. pegFriction -- friction between pegs and snake segments**
% 10. kNseg -- (2n-1) * (n = desired number of body segments)
% 11. lx = dimension of snake segment along direction of motion**
% 12. ly = dimension of snake segment out of screen
% 13. lz = dimension of snake segment in plane of motion, transverse to initial trajectory**
% 14. boxSx = peg diameter**
% 15. kNpeg = number of pegs
% 16. wpz -- peg z position (offset)**

% 17. dt -- array of time steps for parametric sweep
% 18. iter -- array of number of iterations for parametric sweep
% 19. timeIntegrationType -- (0): Anitescue, (1): Tasora, (2): HHT, (3):
%       Newmark, (4): RK45, (5): Trapezoidal, (6): Implicit Euler 
% 20. recoverySpeed -- contact recovery speed
% 21. colEnvelop -- collision envelop (outward)
% 22. colMargine -- collision margine (inward)

%** = iterate over these

% Indicies (Identifiers): 
%                       Robot: {0, 1, 2, ..., (N_Seg -1)}                    
%                       Robot Head and Tail: (N_Seg): Head, (N_Seg+1): Tail
%                       Ground: -1
%                       Pegs: -2, -3, -4, ....
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ./SnakeProject 25 0.6 0.2 0.14 1 10 0 1.70 0 25 0.2 0.1 0.1 0.1 51 1
KK = 13;    % needs to be the same as kNseg
AA = 0.64; %0.4 * pi / 2;
ww = 0.15;
sp = 0.061; % peg spacing
wpx = 1.5;
wpz = 0;
stopTime = 40;
saveVideo = 0;
RFTstartTime = 2.65; 
pegFriction = 0.4;
kNseg = 13;
lx = 0.056; % robot length
ly = 0.035;     % robot height
lz = 0.038;     % robot width
boxSx = 0.045;  % peg diameter
kNpeg = 0;%1;
multSin = 1; % 0.1 % soil model sin term prefactor
multCos = 50.0; % 5.0 % soil model cos term prefactor

load('C:\Users\Arman\Documents\Repos\Git\snake\matlab\ExperimentalData\positionAndForceDataOnPeg.mat');

xMe = mean(xc(1,:));
zMe = mean(zc(1,:));
dXe = xpegs - xMe;
dZe = zpegs - zMe;

xMs15=0.864786876666667;     % CM, 15 components
zMs15=-0.013183152082000;
xMs13=0.863124194384616;     % CM, 15 components
zMs13=0.002312207366923;
xMs=xMs13;     % CM
zMs=zMs13;
x13s = 1.068885630000000;  % head segment
z13s = -0.115747901000000;
x14s = 1.088547390000000;  % head extera half sphere piece
z14s = -0.109945727000000;

% xOnePeg = 1.361887876666667; 
% zOnePeg = -0.004578152082000; 
% xOnePeg = xMs + dZe; 
% zOnePeg = zMs + dXe; 
xOnePeg = xMs + zpegs; 
zOnePeg = zMs + xpegs;

% xPurturbPeg = [-1.75:0.05:.25] * boxSx;
% zPurturbPeg = [-3.25:0.1:.75] * boxSx;
% xPurturbPeg = [-.2:0.01:.2] * boxSx;
% zPurturbPeg = [-.2:0.01:.2] * boxSx;
% xPurturbPeg = [-.05] * boxSx;
% xPurturbPeg = [-1] * boxSx;
xPurturbPeg = 0.5665; %0.1665; %3.7 * boxSx = 0.1665;
zPurturbPeg = -0.009; %-0.2 * boxSx = 0.009;

% % Save library paths
% MatlabPath = getenv('LD_LIBRARY_PATH');
% % Make Matlab use system libraries
% setenv('LD_LIBRARY_PATH',getenv('PATH'))

plotColor={'-.r','-.b','-.k','.-m','-.g'};
% dt = [0.01,.003,.001,.0003,.0001];
% iter = [50,200,500];
% timeIntegrationType = [0,1];
% recoverySpeed = [0.1, 0.3, 0.6, 0.9, 1.2];
% colEnvelop = [0.01, .03, .05];
% colMargine = [.005,.01,.03];

dt = .01;%.002;
iter = 500;
timeIntegrationType = 1;
recoverySpeed =0.6;
colEnvelop = .03;
colMargine = .01;

xdata={};
zdata={};
mTimeData={};
fxSnakeData={};
fzSnakeData={};
fxPegsData={};
fzPegsData={};
fTimeData={};
myLegend={};
myTitle={};
'yo1'
xOnePeg + xPurturbPeg(1)
'yo2'
zOnePeg + zPurturbPeg(1)
% for dt=[.1,.01,.003,.001,.0003,.0001]
for i_purtX=1:size(xPurturbPeg,2)
    for i_purtZ=1:size(zPurturbPeg,2)
        tic
        system(['SnakeProject.exe ', num2str(KK), ' ', num2str(AA),  ' ', num2str(ww), ' ',...
            num2str(sp), ' ', num2str(wpx), ' ', num2str(stopTime),...
            ' ', num2str(saveVideo), ' ',num2str(RFTstartTime),' ',num2str(pegFriction),' ',...
            num2str(kNseg),' ',num2str(lx),' ',num2str(ly),' ',num2str(lz),' ',num2str(boxSx),' ',...
            num2str(kNpeg),' ', num2str(wpz),' ',...
            num2str(dt),' ', num2str(iter),' ',num2str(timeIntegrationType), ' ',...
            num2str(recoverySpeed),' ',num2str(colEnvelop),...
            ' ',num2str(colMargine), ' ', num2str(multSin), ' ', num2str(multCos), ...
            ' ', num2str(xOnePeg + xPurturbPeg(1)), ' ', num2str(zOnePeg + zPurturbPeg(1))]);
        fTime = toc
        sprintf('simulation time %d\n', fTime);


        movdata = importdata('snake.mov');
        id = movdata(:,1); % identifies body segment
        numSegments = length(unique(id));%max(id)-min(id(id>=0))+1;
        x = reshape(movdata(:,2),numSegments,size(movdata,1)/numSegments);
        z = reshape(movdata(:,4),numSegments,size(movdata,1)/numSegments);
        mTime = reshape(movdata(:,15),numSegments,size(movdata,1)/numSegments);

        xdata{i_purtX,i_purtZ} = x;
        zdata{i_purtX,i_purtZ} = z;
        mTimeData{i_purtX,i_purtZ} = mTime(1,:); % they are all the same
        id = reshape(id,numSegments,size(movdata,1)/numSegments);

        [fxSnake,fzSnake,fxPegs,fzPegs,idSnake,idPegs,fTime] = importForceData('','snake.cot',RFTstartTime,dt); 
        fxSnakeData{i_purtX,i_purtZ}=fxSnake;
        fzSnakeData{i_purtX,i_purtZ}=fzSnake;
        fxPegsData{i_purtX,i_purtZ}=fxPegs;
        fzPegsData{i_purtX,i_purtZ}=fzPegs;
        fTimeData{i_purtX,i_purtZ}=fTime;
    end
end

nameCount=0;
snakeSegmentPlot = 7; %kNseg; 
pegSegmentPlot = 1;
for i_iter=1:size(iter,2)
    for i_tIntType=1:size(timeIntegrationType,2)
        for i_recSpeed=1:size(recoverySpeed,2)
            for i_env=1:size(colEnvelop,2)
                for i_mar=1:size(colMargine,2)
                    nameCount = nameCount + 1;
%                     %%                
%                     f1=figure
%                     hold on
%                     for i_dt=1:size(dt,2)
%                         moveTime = mTimeData{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar};
%                         indMoveTime = find(moveTime > RFTstartTime-2*dt(i_dt) & moveTime < RFTstartTime+2*dt(i_dt));
%                         tStartInd=floor(mean(indMoveTime));
%                         plot(zdata{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar}(snakeSegmentPlot,tStartInd:end),xdata{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar}(snakeSegmentPlot,tStartInd:end),plotColor{i_dt});
%                     end
%                     plot(xc(:,7),zc(:,7));
%                     hold off
%                     legend(myLegend);
% %                     title(myTitle{i_iter,i_tIntType,i_recSpeed,i_env,i_mar});
%                     print(f1,'-r300','-djpeg ',strcat('pos_', int2str(nameCount) , '.jpg'))  %'-dtiff ' for tiff
%                     saveas(f1,strcat('pos_', int2str(nameCount)),'fig');
                    %% total force
                    f2=figure
                    hold on
                    for i_dt=1:size(dt,2)
                        fx=fxPegsData{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar}(pegSegmentPlot,:);
                        fz=fzPegsData{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar}(pegSegmentPlot,:);
                        f=sqrt(fx.^2+fz.^2);
                        plot(fTimeData{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar},...
                        f,plotColor{i_dt});
                    end
                    hold off
%                     legend(myLegend);
                    title(myTitle{i_iter,i_tIntType,i_recSpeed,i_env,i_mar});
                    xlabel('t (s)');
                    ylabel('Fx (N)');
                    print(f2,'-r300','-djpeg ',strcat('force_', int2str(nameCount) , '.jpg'))  %'-dtiff ' for tiff 
                    saveas(f2,strcat('force_', int2str(nameCount)),'fig');
%                     %% force z
%                     f3=figure
%                     hold on
%                     for i_dt=1:size(dt,2)
%                         plot(fTimeData{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar},...
%                         fzPegsData{i_dt,i_iter,i_tIntType,i_recSpeed,i_env,i_mar}(pegSegmentPlot,:),plotColor{i_dt});
%                     end
%                     hold off
%                     legend(myLegend);
%                     title(myTitle{i_iter,i_tIntType,i_recSpeed,i_env,i_mar});
%                     xlabel('t (s)');
%                     ylabel('Fz (N)');
%                     print(f3,'-r300','-djpeg ',strcat('force_', int2str(nameCount) , '.jpg'))  %'-dtiff ' for tiff 
%                     saveas(f3,strcat('force_', int2str(nameCount)),'fig');
                end
            end
        end
    end
end

%% figuring out the initial position of the robot at rftStartTime
t = mTimeData{1,1,1,1,1,1};
ind = find(t>RFTstartTime-3.5 * dt & t < RFTstartTime + 3.5 * dt); 
xMs = mean(x(1:13,265));
zMs = mean(z(1:13,265));
% for initial pos
% Reassign old library paths
% setenv('LD_LIBRARY_PATH',MatlabPath)
