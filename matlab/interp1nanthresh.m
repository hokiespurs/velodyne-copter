function yi = interp1nanthresh(x,y,xi,maxdx,maxdy,varargin)
% INTERP1NANTHRESH interpolates over nans less than the provided thresholds
%   This function uses interp1 to interpolate over nans, but it only
%   interpolates over the nan values if the difference in x and y values
%   are both less than the provided thresholds.  The threshold is set as
%   anything less than or equal to the value.
%
%   When there is a duplicate X value, the Y values are averaged
% 
% 
%   varargin can be used to pass the same things that would normally be
%   passed to interp1.  these thresholds will NOT apply to extrapolation
%
% Inputs:
%   - x     : vector of x sample points
%   - y     : vector of y sample data
%   - xi    : vector of x query points
%   - maxdx : maximum distance to interpolate over in x dimension
%   - maxdy : maximum distance to interpolate over in y dimension
% 
% Outputs:
%   - yi    : interpolated y values 
% 
% Examples:
%     x = [1 2 3 7 8 9 14 20 21];
%     y = [1 2 3 7 8 9 14 20 19];
%     xi = 0:21;
%     MAXDX = 6;
%     MAXDY = 6;
%     yi = interp1nanthresh(x,y,xi,MAXDX,MAXDY);
% 
% Dependencies:
%   - n/a
% 
% Toolboxes Required:
%   - n/a
%
% Author        : Richie Slocum    
% Email         : richie@cormorantanalytics.com    
% Date Created  : 14-Jun-2017    
% Date Modified : 14-Jun-2017     
% Github        : https://github.com/hokiespurs/general-purpose-matlab

% convert to 2D Array Of column vectors with no nans
x = x(:);
y = y(:);
nanvals = isnan(x) | isnan(y);
xy = [x(~nanvals) y(~nanvals)];

% remove duplicates
[xvals,~,idx] = unique(xy(:,1));
if numel(xvals)~=size(xy,1) % duplicates exist and need to be averaged
    xy = [xvals accumarray(idx,xy(:,2),[],@mean)];
end
npts = size(xy,1);

% return all nans if the data is all nans
if npts==0
    yi = nan(size(xi));
    return
end

% sort data
xy = sortrows(xy,1);

% find bad jumps in x and y
badX = diff(xy(:,1))>maxdx;
badY = diff(xy(:,2))>maxdy;
badVals = badX | badY;

nBadVals = sum(badVals);

% put nans in between bad gaps
ynew = nan(npts+nBadVals,1);

oldind = (1:npts)';
newind = oldind + [0; cumsum(badVals)];

xnew=interp1(newind,xy(:,1),1:npts+nBadVals);
ynew(newind)=xy(:,2);

% interpolate
yi = interp1(xnew,ynew,xi,varargin);

end