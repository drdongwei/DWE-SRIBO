function x = progation_lineardrag(xp, u, dt,kappa)
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt;
     0, 0, 0, 1-kappa(1)*dt, 0, 0;
     0, 0, 0, 0, 1-kappa(2)*dt, 0;
     0, 0, 0, 0, 0, 1-kappa(3)*dt;
     ];
 
B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;
     ];
% xp
% u
x = A * xp + B * u;

