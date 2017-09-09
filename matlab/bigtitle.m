function [htxt,h]= bigtitle(titlestr,xpos,ypos,varargin)
% BIGTITLE generates a big title anywhere on the screen
%   bigtitle is used to make large titles over multiple subplots.  An
%   invisible axes is created, and the text is overlaid in positions
%   relative to the 'normalized' units of the figure frame.
% 
% Inputs:
%   - titlestr : desired string for the title
%   - xpos     : Normalized x position for the text to be centered on
%   - ypos     : Nromalized y position for the text to be placed above
%   - varargin : Extra parameters which may be passed to the 'text' command
% 
% Outputs:
%   - htxt : handle for the title txt box
%   - h    : handle for the invisible axes containing the title
% 
% Examples:
%     figure(1);clf;
%     subplot(2,2,1);subplot(2,2,2);
%     subplot(2,2,3);subplot(2,2,4);
%     bigtitle('This is a BIG title',0.5,0.93,'interpreter','latex');
%     bigtitle('This is also a BIG title!',0.5,0.46,'fontsize',18);
% 
% Dependencies:
%   - n/a
% 
% Toolboxes Required:
%   - n/a
% 
% Author        : Richie Slocum
% Email         : richie@cormorantanalytics.com
% Date Created  : 20-Jun-2017
% Date Modified : 20-Jun-2017  
% Github        : https://github.com/hokiespurs/general-purpose-matlab


h = axes('units','normalized','pos',[0 0  1 1],...
    'visible','off','HitTest','off');
htxt = text(xpos,ypos,titlestr,'parent',h,...
    'HorizontalAlignment','center','VerticalAlignment','bottom',...
    varargin{:});
end
