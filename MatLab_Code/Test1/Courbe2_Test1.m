F_C=[2,4,6,8,10,12,14,17.60]; % Force (à partir du contact, 2N)
F_RP=[2,4,6,6.93];
D_C=[0,1.31,2.39,3.77,4.47,5.45,6.42,7.20]; % Deflection relative au contact (2N)
D_RP=[0,0.76,1.46,1.89];

figure('Color','w','Units','normalized','Position',[0.1 0.1 0.6 0.6]);
plot(F_C,D_C,'b-o','LineWidth',2,'DisplayName','Compliant Gripper');
hold on;
plot(F_RP,D_RP,'r-s','LineWidth',2,'DisplayName','R\&P Gripper');
grid on;
xlabel('Applied Force (N)','FontSize',12,'Interpreter','latex');
ylabel('Finger Deflection  (mm)','FontSize',12,'Interpreter','latex');
title('Effective Compliance (Zero at 2N)','FontSize',14,'Interpreter','latex');
legend('Location','southeast','Interpreter','latex');
set(gca,'FontSize',11,'TickLabelInterpreter','latex');
print('stiffness_graph','-dpng','-r300');