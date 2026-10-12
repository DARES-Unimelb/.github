function out = v_sim_v11_models(varargin)
% V_SIM_V11_MODELS V11: every .slx on disk contains the blocks the build recorded.
%
%   out = v_sim_v11_models()
%
% Independent of the generator: the census is read from
% results/sim_build_census.mat as the builders left it, and each model is closed
% first and then loaded from its file, so what is counted is what is on disk and
% not what happens to be in memory. A mismatch asserts, it does not warn.
%
% TWO INDEPENDENT DATA SOURCES, AND THIS FILE IS ONLY ONE OF THEM. The census is
% written by the generators, build_uacs_models.m for uacs_plant, uacs_manual_pid
% and uacs_lqr, and sim_build_plant_ref.m for uacs_plant_ref. This verifier reads
% that file and never writes it. A check that produces the data it checks is
% checking itself, and the whole value of V11 is that the expected counts come
% from a build that ran at a different time and did not know it would be audited.
% Do not make this function register anything, not even to "fix" a missing entry:
% a missing entry is a failed check, not a gap to be filled in.
%
% The census is merged rather than overwritten by both generators, so either one
% running alone leaves the other's record intact. Every field of census that is
% not 'stamp' or 'blk_kind' is treated as one model.

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
   % exist(...,'file') returns 4, not 2, for a .slx: MATLAB classifies it as a
   % Simulink model rather than a plain file. Asserting == 2 reported a false
   % "not on disk" for models that were present and loadable, so both codes are
   % accepted here.
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
