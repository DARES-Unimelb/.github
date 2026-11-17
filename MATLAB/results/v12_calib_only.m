% V12_CALIB_ONLY Re-run the calibration alone, so the ISA altitude sweep is
% measured without spending a whole cross-check job on it.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'params'), fullfile(root, 'plant'), fullfile(root, 'control'), ...
   fullfile(root, 'sim'), fullfile(root, 'run'), fullfile(root, 'verify'), '-begin');
clear functions; rehash path;
[P, src] = sim_init_params();
sim_deps_reset();
sim_plant_source('eom_6dof');
assignin('base', 'sim_ts_ctrl', P.sim.Ts_ctrl);
assignin('base', 'sim_t_end', P.sim.t_end_s);
tr = sim_v12_trim(P);
assignin('base', 'trim', tr);
assignin('base', 'sim_x0', tr.x_star(:));
fprintf('self check %s, revision %d\n', src, P.meta.revision);
C = sim_ref_calibrate(P, struct('t_end', 2.0));
fprintf('calibration PASS %d\n', C.pass);
fprintf('ISA sweep %d points: identical rho %d, pos %d/%d, neg %d/%d\n', C.isa.n_sweep, ...
   C.isa.own_n_identical_rho, C.isa.pos_identical_rho, C.isa.pos_n, ...
   C.isa.neg_identical_rho, C.isa.neg_n);
fprintf('ISA max abs rho %.6e at h %.2f, max rel %.6e\n', C.isa.own_max_abs_rho, ...
   C.isa.own_arg_max_abs_rho, C.isa.own_max_rel_rho);
disp('CALIB ONLY DONE');
