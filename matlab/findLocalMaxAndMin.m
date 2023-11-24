function [maxIdx,minIdx,maxVals,minVals] = findLocalMaxAndMin(x,threshold,includeEndPoints,dx)
    
    if nargin < 2 || isempty(threshold)
        threshold = 1e20;
    end
    
    if nargin < 3 || isempty(includeEndPoints)
        includeEndPoints = false;
    end
    
    if nargin < 4 || isempty(dx)
        dx = 1;
    end
    
    threshold = threshold/dx;
    xIdxKeep = find(~isnan(x) & ~isinf(x));
    x = x(xIdxKeep);
    
    maxIdx = find(imregionalmax(x));
    minIdx = find(imregionalmin(x));

    if ~includeEndPoints
        N = length(x);
        maxIdx = setdiff(maxIdx,[1 N]);
        minIdx = setdiff(minIdx,[1 N]);
    end
    
    maxVals = x(maxIdx);
    minVals = x(minIdx);
    
    diffIdxMax = diff(maxIdx);
    diffIdxMin = diff(minIdx);
    
    while min(diffIdxMax) < threshold
        keep = true(size(maxVals));
        idx = find(diffIdxMax < threshold);
        for i=1:length(idx)
            currentIdx = maxIdx([idx(i) idx(i)+1]);
            [~,currentMinIdx] = min(x(currentIdx));
            if currentMinIdx == 1
                keep(idx(i)) = false;
            else
                keep(idx(i)+1) = false;
            end
        end
        maxIdx = maxIdx(keep);
        maxVals = maxVals(keep);
        diffIdxMax = diff(maxIdx);
    end
    
    
    while min(diffIdxMin) < threshold
        keep = true(size(minVals));
        idx = find(diffIdxMin < threshold);
        for i=1:length(idx)
            currentIdx = minIdx([idx(i) idx(i)+1]);
            [~,currentMaxIdx] = max(x(currentIdx));
            if currentMaxIdx == 1
                keep(idx(i)) = false;
            else
                keep(idx(i)+1) = false;
            end
        end
        minIdx = minIdx(keep);
        minVals = minVals(keep);
        diffIdxMin = diff(minIdx);
    end
    
    maxIdx = xIdxKeep(maxIdx);
    minIdx = xIdxKeep(minIdx);
    
    