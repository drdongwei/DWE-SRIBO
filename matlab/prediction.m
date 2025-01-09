function x = prediction(x0, u, dt, prog)
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
 
B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];
 
xt(:,1) = x0;
kappa=[0.2;0.2;2];

for i = 1:length(u(1,:))-1
    xt(:, i+1) = prog(xt(:, i),u(:, i),dt);
end
x = xt;