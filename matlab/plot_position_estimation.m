f=figure(1);
SetDefaultFigureStyle(f);
time0 = time(1);
plot(time(1:length(xt(1,:)))-time0,xt(1,:),'r', 'linewidth', 1)

plot(time(1:length(xt(2,:)))-time0,xt(2,:),'m', 'linewidth', 1)
plot(time(1:length(xt(3,:)))-time0,xt(3,:),'b', 'linewidth', 1)

plot(time(1:length(xt(1,:)))-time0,gtd(1,1:length(xt(1,:))),'r--', 'linewidth', 1)
plot(time(1:length(xt(2,:)))-time0,gtd(2,1:length(xt(2,:))),'m--', 'linewidth', 1)
plot(time(1:length(xt(3,:)))-time0,gtd(3,1:length(xt(3,:))),'b--', 'linewidth', 1)
legend('$\hat{x}$','$\hat{y}$','$\hat{z}$','$\hat{x}_g$','$\hat{y}_g$','$\hat{z}_g$','Interpreter','LaTex','NumColumns',6) 
xlabel('$t$ (s)','Interpreter','latex')
ylabel('$p$ (m)','Interpreter','latex')
ylim([-2,5])