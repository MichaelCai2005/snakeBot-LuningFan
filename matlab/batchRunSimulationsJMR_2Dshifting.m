function batchRunSimulationsJMR_2Dshifting(widths,lengths,amplitudes,spacings,diameters,frictions,runParameters)

% use current time to define seed for random number generation
% now = datetime;
% nowVec = datevec(now);
% yyyy = num2str(nowVec(1));
% yy = yyyy(end-1:end);
% mm = num2str(nowVec(2),'%02.0f');
% dd = num2str(nowVec(3),'%02.0f');
% hh = num2str(nowVec(4),'%02.0f');
% seed = str2double([yy mm dd hh]);


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
%  9. pegFriction -- friction between pegs and snake segments
% 10. kNseg -- (2n-1) * (n = desired number of body segments)
% 11. lx = dimension of snake segment along direction of motion**
% 12. ly = dimension of snake segment out of screen
% 13. lz = dimension of snake segment in plane of motion, transverse to initial trajectory**
% 14. boxSx = peg diameter**
% 15. kNpeg = number of pegs

%** = iterate over these
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if nargin < 7 || isempty(runParameters)
    % define a few run parameters that are not iterated over
    runParameters.ww = 0.2; %omega: temporal frequency
    %     runParameters.seed = seed;
    runParameters.dt = 0.01;
    runParameters.stopTime = 40;
    runParameters.saveVideo = 1;
    runParameters.kNseg = 13+12;%50+49;%9+8;
    runParameters.iterationsPerSetting = 1;
    runParameters.filePrefix = 'OneWave_PositionControl_LateralAndForeAft_cos5_sin0';%'OneWave_4Peg_TorqueControl_P1.1_I0_D0_angle-0.34_cos5_sin0';
    runParameters.saveFolder = ['D:\Jennifer\2Dshift\51pegs\AA0.4_2\'];%\expTorqueControl\'];%experimentalParameterSweep\' num2str(seed) '\'];
    runParameters.workingDir =  'D:\Simulation\src\build\snake_sim\RelWithDebInfo\';%'D:\Simulation\perfectControlWith2Dshifting\InfAnisotropy_noFriction\'; %'D:\Simulation\src\build\snake_sim\RelWithDebInfo\';%'D:\Jennifer\Dropbox\Goldman\snakeSimulation\51peg\2Dshift\code\';%'D:\Simulation\src\build\snake_sim\RelWithDebInfo\';
    runParameters.KK = 25;
    runParameters.kNpeg = 51;
    runParameters.lateralVariation = true;
    runParameters.foreAftVariation = true;
    runParameters.lateralOffsets = [];
    runParameters.rftStartTime = 1.70;%0;
    %runParameters.zOffset = 0.45;
    runParameters.angleOffset = 0;%-0.364;
    runParameters.rftT0 = 1.70;
    runParameters.seed = 16043021;
end
    
    runParameters.DataDestDir = runParameters.saveFolder;%strcat(runParameters.saveFolder,'Snake simulation data\');
    mkdir(runParameters.DataDestDir);
    
    
    numAmplitudes = length(amplitudes);
    numWidths = length(widths);
    numLengths = length(lengths);
    numSpacings = length(spacings);
    numDiameters = length(diameters);
    
    RFTstartTimes = zeros(numAmplitudes,numWidths,numLengths);
    slopes = zeros(numAmplitudes,numWidths,numLengths);
    dx = zeros(numAmplitudes,numWidths,numLengths);
    dz = zeros(numAmplitudes,numWidths,numLengths);
    
    
    cd(runParameters.workingDir) % go into correct directory
    
    % rng(seed); % set the seed for the random number generation
    % randomVector = rand(runParameters.iterationsPerSetting,1);
    
    
    for A = 1:numAmplitudes
        AA = amplitudes(A);
        
        for W = 1:numWidths
            lz = widths(W);
            ly = widths(W);
            
            for L = 1:numLengths
                lx = lengths(L);
                if isempty(runParameters.rftStartTime)
                    
                    [RFTstartTimes(A,W,L),slopes(A,W,L),dx(A,W,L),dz(A,W,L)] = findMinVerticalDisplacement(runParameters.KK,AA,runParameters.ww,runParameters.stopTime,runParameters.kNseg,lx,ly,lz,runParameters.rftT0);
                else RFTstartTimes(A,W,L) = runParameters.rftStartTime;
                end
                for D = 1:numDiameters
                    boxSx = diameters(D);
                    for S = 1:numSpacings
                        sp = spacings(S);
                        mkdir([runParameters.DataDestDir num2str(sp,'%0.2f') '\']);
                        for F = 1:length(frictions)
                            friction = frictions(F);
                            
                            % set seed for random number generation
                            now = datetime;
                            nowVec = datevec(now);
                            yyyy = num2str(nowVec(1));
                            yy = yyyy(end-1:end);
                            mm = num2str(nowVec(2),'%02.0f');
                            dd = num2str(nowVec(3),'%02.0f');
                            hh = num2str(nowVec(4),'%02.0f');
                            
                            if isempty(runParameters.seed)
                                seed = str2double([yy mm dd hh]);
                            else seed = runParameters.seed;
                            end
                            
                            rng(seed); % set the seed for the random number generation
                            %randomVector = rand(runParameters.iterationsPerSetting,2);
                            randomVector = rand(1000,2);
                            %lambda = WaveLength(lx,(runParameters.kNseg+1)/2, AA, runParameters.KK);
                            
                            if runParameters.lateralVariation && ~runParameters.foreAftVariation
                                deltaX = 0;
                                deltaZ = dz(A,W,L);
                            elseif ~runParameters.lateralVariation && runParameters.foreAftVariation
                                deltaX = dx(A,W,L);
                                deltaZ = 0;
                            elseif runParameters.lateralVariation && runParameters.foreAftVariation
                                deltaZ = dz(A,W,L);
                                deltaX = dx(A,W,L);
                            end
                            
                            wallPos = (randomVector(:,1)*(deltaX));%+lx/2+lz/2));
                            if runParameters.kNpeg == 1 % if one peg, use snake information
                                
                                zOffset = ((randomVector(:,2)-0.5)*(deltaZ+lz));
                            else %use peg information
                                zOffset = ((randomVector(:,2)-0.5)*(sp+boxSx));
                            end
                            
                            wallPosUnique = unique(wallPos);
                            zOffsetUnique = unique(zOffset);
                            
                            for X = 98:98%length(wallPosUnique)
                                wpx = wallPos(X);
                                %for Z = 1:length(zOffsetUnique)
                                
                                %    if runParameters.lateralVariation
                                iterationNumber = X;
                                % else  iterationNumber = X;
                                %end
                                
                                
                                runChrono(runParameters.KK,AA,runParameters.ww,sp,wpx,zOffset(X),runParameters.stopTime,runParameters.saveVideo,RFTstartTimes(A,W,L),friction,runParameters.kNseg,lx,ly,lz,boxSx,runParameters.kNpeg);
                                %
                                %                             system(['main.exe ', num2str(runParameters.KK), ' ', num2str(AA),  ' ', num2str(runParameters.ww), ' ',num2str(sp), ' ', num2str(wpx), ' ', num2str(runParameters.stopTime),...
                                %                                 ' ', num2str(runParameters.saveVideo), ' ',num2str(RFTstartTimes(A,W,L)),' ',num2str(runParameters.pegFriction),' ',num2str(runParameters.kNseg),' ',...
                                %                                 num2str(lx),' ',num2str(ly),' ',num2str(lz),' ',num2str(boxSx),' ',num2str(runParameters.kNpeg)]);
                                %
                                
                                
                                newName = strcat('snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(iterationNumber,'%04i'),'_seed',num2str(seed,'%10i')); %rename
                                
                                %movefile([runParameters.workingDir 'snake.mov'],[runParameters.DataDestDir newName '.mov']); %rename
                                %movefile([runParameters.workingDir 'snake.jnt'],[runParameters.DataDestDir newName '.jnt']); %rename
                                %movefile([runParameters.workingDir 'snake.cot'],[runParameters.DataDestDir newName '.cot']); %rename
                                
                                % not keeping original output files
                                
                                getPositionAndSpeedData(runParameters.workingDir,'snake.mov',ceil(RFTstartTimes(A,W,L)/runParameters.dt),[runParameters.DataDestDir num2str(sp,'%0.2f') '\'],newName);
                                importForceData(runParameters.workingDir,'snake.cot',RFTstartTimes(A,W,L),runParameters.dt,[runParameters.DataDestDir num2str(sp,'%0.2f') '\'],newName);
                                
                                
                                %                             %move data files
                                %                             movefile('snake.mov',strcat('snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slope',num2str(slopes(A,W,L),'%0.5f'),'_dxHeadPerCycle',num2str(dx(A,W,L),'%2.4f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(X,'%04i'),'_seed',num2str(seed,'%10i'),'.mov')); %rename
                                %                             movefile('snake.jnt',strcat('snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slope',num2str(slopes(A,W,L),'%0.5f'),'_dxHeadPerCycle',num2str(dx(A,W,L),'%2.4f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(X,'%04i'),'_seed',num2str(seed,'%10i'),'.jnt')); %rename
                                %                             movefile('snake.cot',strcat('snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slope',num2str(slopes(A,W,L),'%0.5f'),'_dxHeadPerCycle',num2str(dx(A,W,L),'%2.4f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(X,'%04i'),'_seed',num2str(seed,'%10i'),'.cot')); %rename
                                %                             snakemovfile=strcat(runParameters.workingDir,'snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slope',num2str(slopes(A,W,L),'%0.5f'),'_dxHeadPerCycle',num2str(dx(A,W,L),'%2.4f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(X,'%04i'),'_seed',num2str(seed,'%10i'),'.mov');
                                %                             snakejntfile=strcat(runParameters.workingDir,'snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slope',num2str(slopes(A,W,L),'%0.5f'),'_dxHeadPerCycle',num2str(dx(A,W,L),'%2.4f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(X,'%04i'),'_seed',num2str(seed,'%10i'),'.jnt');
                                %                             snakecotfile=strcat(runParameters.workingDir,'snake_',runParameters.filePrefix,'_Lx',num2str(lx,'%0.3f'),'_Lz',num2str(lz,'%0.3f'),'_pegDiameter',num2str(boxSx,'%0.3f'),'_muPeg',num2str(friction,'%0.2f'),'_rftStart',num2str(RFTstartTimes(A,W,L),'%0.2f'),'_slope',num2str(slopes(A,W,L),'%0.5f'),'_dxHeadPerCycle',num2str(dx(A,W,L),'%2.4f'),'_slitwidth',num2str(sp,'%0.2f'),'_AA',num2str(AA,'%0.3f'),'_KK',num2str(runParameters.KK,'%0.3f'),'_iteration',num2str(X,'%04i'),'_seed',num2str(seed,'%10i'),'.cot');
                                %
                                %                             movefile(snakemovfile,runParameters.DataDestDir);  %move to new folder
                                %                             movefile(snakejntfile,runParameters.DataDestDir);  %move to new folder
                                %                             movefile(snakecotfile,runParameters.DataDestDir);  %move to new folder
                                %end
                            end
                        end
                    end
                end
            end
        end
    end
    
    
    
    save([runParameters.DataDestDir 'runParameters.mat'],'runParameters','widths','amplitudes','lengths','spacings','diameters','RFTstartTimes','slopes','frictions')
    
    return