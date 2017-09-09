function loopStatus(startTime,curLoopNum,nLoops,nskip)
% LOOPSTATUS outputs status of for loop to the command line
%   Outputs the current loop number, the current time, and the expected
%   time the loop will finish to the Matlab command line.  This is useful
%   when running long for loops, to estimate how far along they are.  
%
%   Simply add a 'startTime = now'; command before the loop, and then add 
%   this function at the end of the loop.
%
%   Note: The estimate relies on the for loop remaining relatively constant
%   throughout the duration of the collect.
% 
% Inputs:
%   - startTime  : 1 : datenum : matlab datenum of time when the loop began
%   - curLoopNum : 1 : int     : the current loop number
%   - nLoops     : 1 : int     : the total number of loops
%   - nskip      : 1 : int     : the total status lines to skip
% 
% Outputs:
%   - n/a 
% 
% Examples:
%   NLOOPS = 1000;
%   NSKIPS = 10;
%   startTime = now;
%   for i=1:NLOOPS
%       pause(.05)
%       loopStatus(startTime,i,NLOOPS,NSKIPS)
%   end
%   datestr(now)
%
% Dependencies:
%   - n/a
% 
% Toolboxes Required:
%   - n/a
% 
% TODO:
% 
% Author        : Richie Slocum
% Email         : richie@cormorantanalytics.com
% Date Created  : 14-Apr-2016
% Date Modified : 14-Apr-2016  
% Github        : https://github.com/hokiespurs/general-purpose-matlab

if nargin==3
    nskip = 1; %default to output every loop
end

if nskip==0
   nskip=1; % mod(n,0) = n ...this wont work
end

if mod(curLoopNum,nskip)==0
    elapsedTime = now - startTime; %time it took for the first nLoops
    percentDone = curLoopNum/nLoops;
    
    estimTotalTime = elapsedTime/percentDone; 
    estimTime = estimTotalTime - elapsedTime;
    estimDatenum = datestr(now + estimTime);
    
    ndigits = ceil(log10(nLoops))+1;
    fprintf('%*.0f/%*.0f \t Now: %s \t Expected: %s \t Remaining: %s\n',...
        ndigits,curLoopNum,ndigits,nLoops,datestr(now),estimDatenum, ...
        datestr(estimTime,'dd:HH:MM:SS'));
end

end