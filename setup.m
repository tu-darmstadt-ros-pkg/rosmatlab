[status, result] = system('rospack find rosmatlab');
if status ~= 0
	error('Could not execute rospack to find package rosmatlab.');
end
result(result==10) = [];
addpath([result, '/mex']);

[status, result] = system('rospack plugins --attrib=path rosmatlab');
if status ~= 0
	error('Could not execute rospack to find packages that export matlab.');
end

lines = regexp(result, '\n', 'split');
for i = 1:length(lines)
    if isempty(lines{i}); continue; end
    s = regexp(lines{i}, ' ', 'split');
    disp(['[rosmatlab] Package ' s{1} ' exports ' s{2}]);
    addpath(s{2});
end

clear status
clear result
clear s
clear lines
clear i
