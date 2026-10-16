function summaries = analyze_teleop_export(exportDirectory, outputDirectory, varargin)
% ANALYZE_TELEOP_EXPORT Analyze every arm listed in an export manifest.
manifest = jsondecode(fileread(fullfile(exportDirectory, 'manifest.json')));
assert(manifest.schema_version == 1, 'Unsupported export schema.');
items = struct2cell(manifest.topics);
summaries = cell(size(items));
for k = 1:numel(items)
    item = items{k};
    if item.samples == 0
        continue
    end
    [~, name] = fileparts(item.csv);
    summaries{k} = analyze_teleop(fullfile(exportDirectory, item.csv), ...
                                  fullfile(outputDirectory, name), varargin{:});
end
end
