function sub = plot(topic, datatype, field, varargin)
%PLOT Summary of this function goes here
%   Detailed explanation goes here

% convert varargin to args struct and plot options
args = struct('Field', {field});
plot_options = {};
i = 1;
if mod(length(varargin),2) ~= 0
    plot_options = varargin(1);
    i = 2;
end

while i <= length(varargin)
    if strcmp(varargin{i}, 'Period')
        args.Period = varargin{i+1};
    else
        plot_options = { plot_options{:}, { varargin{i} }, varargin{i+1} };
        if (~iscell(plot_options{end})); plot_options{end} = { plot_options{end} }; end
    end
    i = i + 2;
end

if (~iscell(args.Field)) args.Field = { args.Field }; end
args.n = length(args.Field);

sub = ros.Subscriber(topic, datatype, 10);
hold on;
colororder = get(gca, 'ColorOrder');
for i = 1:args.n
    args.PlotObject(i) = plot(nan, nan, 'Color', colororder(i,:));
    if isempty(get(args.PlotObject(i), 'DisplayName'))
        set(args.PlotObject(i), 'DisplayName', args.Field{i});
    end
end

if ~isempty(plot_options)
    set(args.PlotObject, plot_options{:});
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

xdata = get(obj, 'XData')';
ydata = get(obj, 'YData')';

x = nan(length(field), 1);
for i = 1:sub.UserData.n
    xdata{i} = [xdata{i} t];
    x(i) = eval(['event.message.' field{i}]);
    ydata{i} = [ydata{i} x(i)];

    % update plot data
    if isfield(sub.UserData, 'Period')
        ydata{i}(:,xdata{i} < xdata{i}(end) - sub.UserData.Period) = [];
        xdata{i}(:,xdata{i} < xdata{i}(end) - sub.UserData.Period) = [];
    end
end
set(obj, {'XData'}, xdata', {'YData'}, ydata');

% set XLim property
ax = get(obj(1), 'Parent');
xlim = get(ax, 'XLim');
if xlim(2) < t
    xlim(2) = t;
    if isfield(sub.UserData, 'Period')
        xlim(1) = xlim(2) - sub.UserData.Period;
    end
    set(ax, 'XLim', xlim);
end

end