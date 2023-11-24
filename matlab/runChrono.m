function [x,z,id] = runChrono(KK,AA,ww,sp,wpx,wpz,stopTime,saveVideo,RFTstartTime,pegFriction,kNseg,lx,ly,lz,boxSx,kNpeg, delT, ...
    n_iter, integratorType, recoverySpeed, col_envelop, col_margin, multSin, multCos)

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

%** = iterate over these
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ./SnakeProject 25 0.6 0.2 0.14 1 10 0 1.70 0 25 0.2 0.1 0.1 0.1 51 1

% % Save library paths
% MatlabPath = getenv('LD_LIBRARY_PATH');
% % Make Matlab use system libraries
% setenv('LD_LIBRARY_PATH',getenv('PATH'))

fps=30;
                        
system(['SnakeProject.exe ', num2str(KK), ' ', num2str(AA),  ' ', num2str(ww), ' ',num2str(sp), ' ', num2str(wpx), ' ', num2str(stopTime),...
    ' ', num2str(saveVideo), ' ',num2str(RFTstartTime),' ',num2str(pegFriction),' ',num2str(kNseg),' ',...
    num2str(lx),' ',num2str(ly),' ',num2str(lz),' ',num2str(boxSx),' ',num2str(kNpeg), ' ', num2str(wpz),' ',...
    num2str(delT), ' ', num2str(n_iter), ' ', num2str(integratorType), ' ', num2str(recoverySpeed), ...
    ' ', num2str(col_envelop), ' ', num2str(col_margin), ' ', num2str(multSin), ' ', num2str(multCos)]);


% % Reassign old library paths
% setenv('LD_LIBRARY_PATH',MatlabPath)

if (saveVideo)
    system(['ffmpeg -framerate ',num2str(fps),' -i video_capture/screenshot%05d.jpeg -c:v libxvid -q 0 video_capture/outVid.avi']);
end

movdata = importdata('snake.mov');
id = movdata(:,1); % identifies body segment
numSegments = length(unique(id));%max(id)-min(id(id>=0))+1;
x = reshape(movdata(:,2),numSegments,size(movdata,1)/numSegments);
z = reshape(movdata(:,4),numSegments,size(movdata,1)/numSegments);
id = reshape(id,numSegments,size(movdata,1)/numSegments);

  
end
