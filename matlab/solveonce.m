function xxe = solveonce(x0, u, y, MHE_flow, dt, kappa, k,useflow)

prog = @progation_lineardrag;

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

AA = A^k;

Qp = [1,0.5,1]; Qv = Qp;
Pp = [1,0.5,1]; Pv = Pp;

P=diag([Pp,Pv])*0.1; Q=diag([Qp,Qv]); wflow = useflow*0.5; R = diag([1, wflow, wflow, wflow*0.02]);

num_state = (length(y)-1)/k;                               %obtain the length of states/measurement
Ex = zeros(13*num_state, 6*(num_state+1));                 %initialize the matrix Ex
Eu = zeros(13*num_state, (3*k+6+1)*num_state);             %initialize the matrix Ex
W = zeros(13*num_state, 13*num_state); 
order = 4;

t_matrix = zeros (6*(order+1),6*(order+1));
for j = 1:6
t_matrix(j,(j-1)*(order+1)+1) = 1;
end

for i = 1:num_state
    Ex((i-1)*6+1:i*6,(i-1)*6+1:i*6) = eye(6);
    Ex((i-1)*6+1+6*num_state:(i)*6+6*num_state,(i-1)*6+1:(i)*6) = - AA;
    Ex((i-1)*6+1+6*num_state:(i)*6+6*num_state,(i)*6+1:(i)*6+6) = eye(6);
    rho = sqrt(x0(1:3,i+1)'*x0(1:3,i+1));
    C = [x0(1:3,i+1)'/rho,zeros(1,3)];
    C(2:4,4:6) = eye(3);
    Ex(num_state*6+6*num_state+i*4-3:num_state*6+6*num_state+i*4,(i)*6+1:i*6+6) = C;
    Eu((i-1)*6+1:6*i,(i-1)*6+1:6*i) = eye(6);
    for j = 1:k
        Eu(6*num_state+(i-1)*6+1:i*6+6*num_state,6*num_state+(i-1)*3*k+(j-1)*3+1:(i-1)*3*k+6*num_state+j*3) = A^(k-j)*B;
    end
    
    Eu((num_state)*12+4*i-3:(num_state)*12+4*i,num_state*3*k+6*num_state+4*i-3:num_state*3*k+6*num_state+4*i) = eye(4); 
    ur(3*k*num_state+6*num_state+i*4-3,1) = y(i*k+1);
    ur(3*k*num_state+6*num_state+i*4-2:3*k*num_state+6*num_state+i*4,1) = MHE_flow(:,i*k+1);
    W((i-1)*6+1:6*i,(i-1)*6+1:6*i) = P;
    W(6*num_state+(i-1)*6+1:6*num_state+i*6,6*num_state+(i-1)*6+1:6*num_state+i*6) = Q;
    W(6*num_state+num_state*6+i*4-3:6*num_state+num_state*6+i*4,6*num_state+num_state*6+i*4-3:6*num_state+num_state*6+i*4) = R;
    for j = 1:order+1
        tt(1,j) = (dt*i*k)^(j-1);
    end
    for j = 1:6
        t_matrix((i)*6+j,(j-1)*(order+1)+1:j*(order+1)) = tt;
    end
end

xtt=x0(:,2:end);

ur(1:num_state*6,1) = xtt(:)';
ur(num_state*6+1:3*k*num_state+num_state*6,1) = u(:);
Ex = Ex * t_matrix;
ar = inv((Ex'*W*Ex))*Ex'*W*Eu*ur;
xxt = t_matrix * ar;

for i=1:num_state
    xxe(1:6,i) = xxt(i*6+1:i*6+6);
end
end