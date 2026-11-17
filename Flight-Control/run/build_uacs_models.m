function rep = build_uacs_models(varargin)
% BUILD_UACS_MODELS Generate, save and census the three Simulink models.
%
%   rep = build_uacs_models()
%   rep = build_uacs_models('blk_kind', 'mbfcn')
%
% Writes to MATLAB/model/
%   uacs_plant.slx        bare 12-state object, actuators plus wind in
%   uacs_manual_pid.slx   PID cascade with stick input
%   uacs_lqr.slx          LQR full-state feedback with integral augmentation
%
% Also writes results/simulink_build_report.txt and results/sim_build_census.mat.
% It does not simulate anything: verify/v_sim_*.m checks the files from disk and the
% run scripts do the simulating.

ip = inputParser;
ip.addParameter('blk_kind', 'sfun', @(x) ischar(x) || isstring(x));
ip.addParameter('quiet', false, @(x) islogical(x) || isnumeric(x));
ip.parse(varargin{:});
blk_kind = char(ip.Results.blk_kind);
quiet = logical(ip.Results.quiet);

root = sim_root();
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
rehash path;

[P_init, params_src] = sim_init_params(); %#ok<ASGLU>
sim_deps_reset();
P = evalin('base', 'P');

if ~strcmpi(blk_kind, 'sfun')
   error('build_uacs_models:BlkKind', ...
      ['blk_kind %s is not implemented: the MATLAB Function block path was ' ...
       'abandoned after it hung the shared MATLAB session on 2026-10-08'], blk_kind);
end

D = sim_load_design(P);
probe = sim_probe_plant(P, D.trim.x_star, D.trim.u_star);
D.src.plant_source = probe.source;
lib = sim_lib_report();
assert(isempty(lib.missing), 'build_uacs_models:Library', ...
   'library blocks not found: %s', strjoin(lib.missing, '; '));
push = sim_push_design_vars(P, D);
S = sim_push_scenario(struct('t_end', P.sim.t_end_s, 'x0', D.trim.x_star));
deps = sim_deps_report(true);

rep = struct();
rep.stamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
rep.root = root;
rep.blk_kind = blk_kind;
rep.params = struct('version', P.meta.version, 'date', P.meta.date, ...
   'b', P.geom.b, 'S', P.geom.S, 'cbar', P.geom.cbar, 'm', P.mass.m, ...
   'span_limit_m', P.geom.span_limit_m, 'V_min_mps', P.limits.V_min_mps, ...
   'self_check', params_src);
rep.design = D.src;
rep.plant_probe = probe;
rep.library = lib;
rep.trim_residual = D.trim.residual;
rep.trim_alpha_deg = D.trim.alpha_deg;
rep.trim_de_deg = D.trim.de_deg;
rep.trim_throttle = D.trim.throttle;
rep.deps = deps;
rep.push_count = push.count;
rep.refmap_linearity = push.refmap.linearity;
rep.scenario_default = S;
rep.models = struct('name', {}, 'file', {}, 'top_count', {}, 'total_count', {}, ...
   'line_count', {}, 'top_names', {}, 'inports', {}, 'outports', {});

builders = {@sim_build_plant, @sim_build_manual_pid, @sim_build_lqr};
for k = 1:numel(builders)
   info = builders{k}(P);
   rep.models(end + 1) = struct('name', info.model, 'file', info.file, ...
      'top_count', info.census.top_count, 'total_count', info.census.total_count, ...
      'line_count', info.census.line_count, 'top_names', {info.census.top_names}, ...
      'inports', info.census.num_inports, 'outports', info.census.num_outports);
   if ~quiet
      fprintf('built %-18s top %3d blocks, %3d total, %3d lines -> %s\n', ...
         info.model, info.census.top_count, info.census.total_count, ...
         info.census.line_count, info.file);
   end
end

% The census file is merged, not overwritten: either generator running alone must
% leave the other's entries intact.
census_file = fullfile(root, 'results', 'sim_build_census.mat');
Cs = struct();
if exist(census_file, 'file')
   loaded = load(census_file);
   if isfield(loaded, 'census') && isstruct(loaded.census)
      Cs = loaded;
   end
end
if ~isfield(Cs, 'census') || ~isstruct(Cs.census)
   Cs.census = struct();
end
for k = 1:numel(rep.models)
   Cs.census.(rep.models(k).name) = rep.models(k);
end
Cs.census.stamp = rep.stamp;
Cs.census.blk_kind = blk_kind;
Cs.rep = rep;
save(census_file, '-struct', 'Cs');

% Unload the models so the rest of the chain reads them from disk.
for k = 1:numel(rep.models)
   if bdIsLoaded(rep.models(k).name)
      close_system(rep.models(k).name, 0);
   end
end

rep.report_file = local_write_report(root, rep);
if ~quiet
   fprintf('build report -> %s\n', rep.report_file);
end
end

%% Local helpers
function s = local_str(st, f)
if isfield(st, f)
   s = st.(f);
else
   s = 'unknown';
end
end

function file = local_write_report(root, rep)
file = fullfile(root, 'results', 'simulink_build_report.txt');
fid = fopen(file, 'w');
if fid < 0
   error('build_uacs_models:Report', 'cannot write %s', file);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'UACS Simulink build report\n');
fprintf(fid, 'generated      %s\n', rep.stamp);
fprintf(fid, 'root           %s\n', rep.root);
fprintf(fid, 'block kind     %s (Level-2 MATLAB S-Function; the MATLAB Function block path was withdrawn)\n', rep.blk_kind);
fprintf(fid, 'MATLAB         %s\n', version());
fprintf(fid, '\n[parameters]\n');
fprintf(fid, 'version        %s (%s)\n', rep.params.version, rep.params.date);
fprintf(fid, 'self check     %s\n', rep.params.self_check);
fprintf(fid, 'mass           %.3f kg\n', rep.params.m);
fprintf(fid, 'span           %.3f m (limit %.3f m)\n', rep.params.b, rep.params.span_limit_m);
fprintf(fid, 'area           %.4f m^2, cbar %.4f m\n', rep.params.S, rep.params.cbar);
fprintf(fid, 'V_min          %.3f m/s\n', rep.params.V_min_mps);

fprintf(fid, '\n[design source]\n');
fprintf(fid, 'branch         %s\n', rep.design.branch);
fprintf(fid, 'trim source    %s\n', local_str(rep.design, 'trim_source'));
fprintf(fid, 'lin source     %s\n', local_str(rep.design, 'lin_source'));
fprintf(fid, 'pid gains      %s\n', local_str(rep.design, 'pid_source'));
fprintf(fid, 'lqr gains      %s\n', local_str(rep.design, 'lqr_source'));
if isfield(rep.design, 'detail')
   fprintf(fid, 'detail         %s\n', rep.design.detail);
end
if isfield(rep.design, 'trim_error')
   fprintf(fid, 'trim error     %s\n', rep.design.trim_error);
end
if isfield(rep.design, 'lin_error')
   fprintf(fid, 'lin error      %s\n', rep.design.lin_error);
end
if isfield(rep.design, 'pid_error')
   fprintf(fid, 'pid error      %s\n', rep.design.pid_error);
end
if isfield(rep.design, 'lqr_error')
   fprintf(fid, 'lqr error      %s\n', rep.design.lqr_error);
end
fprintf(fid, 'fingerprint    %d source files\n', numel(strsplit(rep.design.fingerprint, ' | ')));
fprintf(fid, 'reference map linearity residual %.3e\n', rep.refmap_linearity);

fprintf(fid, '\n[plant probe]\n');
fprintf(fid, 'plant source   %s\n', rep.plant_probe.source);
if rep.plant_probe.ok
   fprintf(fid, 'trim derivative norm %.6e\n', rep.plant_probe.dx_norm);
else
   fprintf(fid, 'eom_6dof FAILED: %s\n', rep.plant_probe.error);
end
fprintf(fid, 'trim residual norm %.6e (trim_source %s)\n', ...
   norm(rep.trim_residual), local_str(rep.design, 'trim_source'));

fprintf(fid, '\n[dependencies]\n');
fprintf(fid, 'plant          %s\n', rep.deps.plant_source);
fprintf(fid, 'air data       %s\n', rep.deps.meas_source);
fprintf(fid, 'allocation     %s\n', rep.deps.alloc_source);
fprintf(fid, 'pilot command  %s\n', rep.deps.pilot_source);
fprintf(fid, 'placeholder    %d\n', rep.deps.placeholder_used);
if rep.deps.smoke_ok == 0
   fprintf(fid, 'smoke test     FAILED: %s\n', rep.deps.smoke_error);
else
   fprintf(fid, 'smoke test     ok\n');
end

fprintf(fid, '\n[library blocks]\n');
fprintf(fid, 'checked        %d paths, %d missing\n', rep.library.checked, ...
   numel(rep.library.missing));
for k = 1:numel(rep.library.missing)
   fprintf(fid, 'missing        %s\n', rep.library.missing{k});
end

fprintf(fid, '\n[models]\n');
for k = 1:numel(rep.models)
   m = rep.models(k);
   fprintf(fid, '%-18s top %3d  total %3d  lines %3d  in %d  out %d\n', ...
      m.name, m.top_count, m.total_count, m.line_count, m.inports, m.outports);
   fprintf(fid, '  file   %s\n', m.file);
   fprintf(fid, '  blocks %s\n', strjoin(m.top_names, ' '));
end

fprintf(fid, '\n[base workspace design variables]\n');
w = evalin('base', 'who');
design = w(startsWith(w, 'sim_'));
fprintf(fid, '%d sim_ variables pushed by sim_push_design_vars and sim_push_scenario\n', numel(design));
fprintf(fid, '%s\n', strjoin(sort(design)', ' '));
end
