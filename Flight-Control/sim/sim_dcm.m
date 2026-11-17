function R = sim_dcm(eul)
% SIM_DCM Body-from-NED direction cosine matrix for Euler angles [phi; theta; psi]

phi = eul(1);
theta = eul(2);
psi = eul(3);
cp = cos(phi); sp = sin(phi);
ct = cos(theta); st = sin(theta);
cs = cos(psi); ss = sin(psi);

R = [ct * cs,                ct * ss,               -st; ...
     sp * st * cs - cp * ss, sp * st * ss + cp * cs, sp * ct; ...
     cp * st * cs + sp * ss, cp * st * ss - sp * cs, cp * ct];
end
