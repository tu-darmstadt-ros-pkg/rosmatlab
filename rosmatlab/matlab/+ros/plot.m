function sub = plot(topic, datatype, field, varargin)
%PLOT Summary of this function goes here
%   Detailed explanation goes here

% convert varargin to args struct and plot options
args = struct('Field', field);
plot_options = {};
i = 1; if mod(i,2) ~= 0; plot_options = varargin(1); i = 2; end
while i <= length(varargin)
    if strcmp(varargin{i}, 'Period')
        args.Period = varargin{i+1};
    else
        plot_options = { plot_options(:), varargin(i:i+1) };
    end
    i = i + 2;
end

sub = ros.Subscriber(topic, datatype, 10);
args.PlotObject = plot(0,0, plot_options{:});
set(args.PlotObject, 'XData', [], 'YData', []);

if isempty(get(args.PlotObject, 'DisplayName'))
    set(args.PlotObject, 'DisplayName', field);
end

sub.UserData = args;
sub.addlistener('Callback', @callback);
sub.start();

end

function callback(sub, event)
obj = sub.UserData.PlotObject;
field = sub.UserData.Field;

% stop subscriber if object handle was deleted
if ~ishandle(obj)
    sub.stop();
    return;
end

% get data
t = 0;
if isfield(event.message, 'header')
    t = event.message.header.stamp;
end
if t == 0
    t = sub.getReceiptTime();
end
x = eval(['event.message.' field]);

% update plot data
xdata = [get(obj, 'XData') t];
ydata = [get(obj, 'YData') x];
if isfield(sub.UserData, 'Period')
    ydata(xdata < xdata(end) - sub.UserData.Period) = [];
    xdata(xdata < xdata(end) - sub.UserData.Period) = [];
end
set(obj, 'XData', xdata, 'YData', ydata);

% set XLim property
ax = get(obj, 'Parent');
xlim = get(ax, 'XLim');
if xlim(2) < t
    xlim(2) = t;
    if isfield(sub.UserData, 'Period')
        xlim(1) = xlim(2) - sub.UserData.Period;
    end
    set(ax, 'XLim', xlim);
end

end