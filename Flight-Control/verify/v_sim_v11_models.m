function out = v_sim_v11_models(varargin)
% V_SIM_V11_MODELS V11: every .slx on disk contains the blocks the build recorded.
%
%   out = v_sim_v11_models()
%
% Reads the census the generators left in results/sim_build_census.mat and never writes
% it. Each model is closed and then loaded from its file, so what is counted is what is
% on disk and not what happens to be in memory. A mismatch asserts, it does not warn.
%
% Every field of census other than 'stamp' and 'blk_kind' is one model.

ip = inputParser;
ip.addParameter('verbose', true, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
verbose = logical(ip.Results.verbose);

root = sim_root();
census_file = fullfile(root, 'results', 'sim_build_census.mat');
assert(exist(census_file, 'file') == 2, 'v_sim_v11:NoCensus', ...
   '%s is missing; run build_uacs_models and sim_build_plant_ref first', census_file);
S = load(census_file);
census = S.census;
names = fieldnames(census);
names = names(~strcmp(names, 'stamp') & ~strcmp(names, 'blk_kind'));

out = struct('model', {}, 'top_count', {}, 'expected_top', {}, 'total_count', {}, ...
   'expected_total', {}, 'names_match', {}, 'pass', {});
for k = 1:numel(names)
   name = names{k};
   exp = census.(name);
   % exist(...,'file') returns 4, not 2, for a .slx: MATLAB classes it as a Simulink
   % model rather than a plain file, so both codes mean "present"
   code = exist(exp.file, 'file');
   assert(code == 2 || code == 4, 'v_sim_v11:MissingFile', ...
      '%s is not on disk (exist returned %d)', exp.file, code);
   if bdIsLoaded(name)
      close_system(name, 0);
   end
   load_system(exp.file);
   c = sim_census(name);
   close_system(name, 0);

   e = struct();
   e.model = name;
   e.top_count = c.top_count;
   e.expected_top = exp.top_count;
   e.total_count = c.total_count;
   e.expected_total = exp.total_count;
   e.names_match = isequal(sort(c.top_names(:)), sort(exp.top_names(:)));
   e.pass = (c.top_count == exp.top_count) && (c.total_count == exp.total_count) ...
      && e.names_match;
   out(end + 1) = e; %#ok<AGROW>

   if verbose
      fprintf('V11 %-18s disk top %3d / expected %3d, total %3d / %3d, names %d -> %s\n', ...
         name, c.top_count, exp.top_count, c.total_count, exp.total_count, ...
         e.names_match, local_verdict(e.pass));
   end
   assert(c.top_count == exp.top_count, 'v_sim_v11:TopCount', ...
      '%s has %d top level blocks on disk, the build recorded %d', ...
      name, c.top_count, exp.top_count);
   assert(c.total_count == exp.total_count, 'v_sim_v11:TotalCount', ...
      '%s has %d blocks on disk, the build recorded %d', ...
      name, c.total_count, exp.total_count);
   assert(e.names_match, 'v_sim_v11:Names', ...
      '%s top level block names differ from the build record', name);
end
end

function s = local_verdict(tf)
if tf
   s = 'PASS';
else
   s = 'FAIL';
end
end
