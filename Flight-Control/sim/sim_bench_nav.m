function info = sim_bench_nav(mdl)
% SIM_BENCH_NAV Navigation and measurement subsystem: state split, air data, instrument taps
%
%   in x 12x1    out fb 9x1 / x 12x1 / inst 7x1
%
% fb is the feedback vector; the order is fixed here and every downstream
% subsystem demuxes it in this order
%   1 phi   2 theta   3 p   4 q   5 r   6 V   7 h   8 alpha   9 beta
%
% h is -x(12): the body z axis points down, so height up is the negated state
%
% inst feeds the canvas instruments
%   1 V [m/s]   2 h [m]   3 alpha [deg]   4 phi [deg]   5 theta [deg]
%   6 stall 0/1   7 r [deg/s]
% Angles are converted to degrees here because the gauges are scaled in degrees
%
% The tap blocks must reach a real sink, or block reduction deletes them and the
% gauges bound to them lose their data, so inst ends at an Outport

sub = [mdl '/导航与测量'];
sim_add_block(mdl, '导航与测量', 'simulink/Ports & Subsystems/Subsystem', [1960 300 2120 500]);
sim_clear_model(sub);

sim_add_block(sub, 'x', 'simulink/Sources/In1', sim_pos(0, 0, 30, 14), 'Port', '1');
sim_add_block(sub, 'x_split', 'simulink/Signal Routing/Demux', sim_pos(1, 0, 40, 260), ...
   'Outputs', '12');
sim_add_block(sub, 'meas', 'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
   sim_pos(1, 3, 100, 40), 'FunctionName', 'sim_meas_sfun');
sim_add_block(sub, 'meas_split', 'simulink/Signal Routing/Demux', sim_pos(2, 3, 40, 90), ...
   'Outputs', '4');
sim_add_block(sub, 'h_neg', 'simulink/Math Operations/Gain', sim_pos(2, 6, 40, 30), ...
   'Gain', 'sim_c_neg1');
sim_add_block(sub, 'fb_mux', 'simulink/Signal Routing/Mux', sim_pos(3, 1, 20, 230), ...
   'Inputs', '9');
sim_add_block(sub, 'fb', 'simulink/Sinks/Out1', sim_pos(4, 1, 30, 14), 'Port', '1');
sim_add_block(sub, 'x_out', 'simulink/Sinks/Out1', sim_pos(4, 3, 30, 14), 'Port', '2');

sim_connect(sub, 'x/1', 'x_split/1', 'x/1', 'meas/1', 'x/1', 'x_out/1');
sim_connect(sub, 'meas/1', 'meas_split/1');
sim_connect(sub, 'x_split/12', 'h_neg/1');

%% Instrument taps
sim_add_block(sub, 'alpha_deg', 'simulink/Math Operations/Gain', sim_pos(3, 8, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'phi_deg', 'simulink/Math Operations/Gain', sim_pos(3, 9, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'theta_deg', 'simulink/Math Operations/Gain', sim_pos(3, 10, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'r_deg', 'simulink/Math Operations/Gain', sim_pos(3, 11, 50, 26), ...
   'Gain', '180/pi');
sim_add_block(sub, 'alpha_abs', 'simulink/Math Operations/Abs', sim_pos(4, 8, 40, 26));
% two-input comparison, not the block's built-in threshold: that parameter name varies between releases
sim_add_block(sub, 'const_stall', 'simulink/Sources/Constant', sim_pos(4, 10, 90, 30), ...
   'Value', 'sim_alpha_stall_abs');
sim_add_block(sub, 'stall_rel', 'simulink/Logic and Bit Operations/Relational Operator', ...
   sim_pos(5, 8, 60, 26), 'Operator', '>');
% the Relational Operator outputs boolean, which a Mux rejects, so convert to double
sim_add_block(sub, 'stall_dbl', 'simulink/Signal Attributes/Data Type Conversion', ...
   sim_pos(6, 8, 60, 26), 'OutDataTypeStr', 'double');
sim_connect(sub, 'meas_split/2', 'alpha_deg/1', 'meas_split/2', 'alpha_abs/1');
sim_connect(sub, 'alpha_abs/1', 'stall_rel/1', 'const_stall/1', 'stall_rel/2');
sim_connect(sub, 'stall_rel/1', 'stall_dbl/1');
sim_connect(sub, 'x_split/7', 'phi_deg/1', 'x_split/8', 'theta_deg/1', ...
   'x_split/6', 'r_deg/1');

sim_add_block(sub, 'inst_mux', 'simulink/Signal Routing/Mux', sim_pos(7, 7, 20, 160), ...
   'Inputs', '7');
sim_add_block(sub, 'inst', 'simulink/Sinks/Out1', sim_pos(8, 7, 30, 14), 'Port', '3');
sim_connect(sub, 'meas_split/1', 'inst_mux/1', 'h_neg/1', 'inst_mux/2', ...
   'alpha_deg/1', 'inst_mux/3', 'phi_deg/1', 'inst_mux/4', ...
   'theta_deg/1', 'inst_mux/5', 'stall_dbl/1', 'inst_mux/6', ...
   'r_deg/1', 'inst_mux/7', 'inst_mux/1', 'inst/1');

%% Feedback vector
sim_connect(sub, 'x_split/7', 'fb_mux/1', 'x_split/8', 'fb_mux/2', ...
   'x_split/4', 'fb_mux/3', 'x_split/5', 'fb_mux/4', 'x_split/6', 'fb_mux/5', ...
   'meas_split/1', 'fb_mux/6', 'h_neg/1', 'fb_mux/7', ...
   'meas_split/2', 'fb_mux/8', 'meas_split/3', 'fb_mux/9', 'fb_mux/1', 'fb/1');

sim_name_line(sub, 'fb_mux/1', 'fb 9x1 [phi theta p q r V h alpha beta]');
sim_name_line(sub, 'inst_mux/1', 'inst 7x1 [V h alpha_deg phi_deg theta_deg stall r_deg]');
sim_name_line(sub, 'meas_split/1', 'V 空速 [m/s]');
sim_name_line(sub, 'h_neg/1', 'h 高度 [m]');

info = struct('name', '导航与测量', 'path', sub);
end
