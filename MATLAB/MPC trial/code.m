m = 1000;
b = 50;

A = -b/m;
B = 1/m;
C = 1;
D = 0;
sys_c = ss(A, B, C, D);

Ts = 0.1;
sys_d = c2d(sys_c, Ts);

mpcobj = mpc(sys_d, Ts, 20, 5);

% Set constraints
mpcobj.MV.Min = -2000;
mpcobj.MV.Max = 3000;

% Penalize large control inputs and rapid rate shifts relative to output tracking
mpcobj.Weights.ManipulatedVariables = 0.01; 
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.Weights.OutputVariables = 1.0;