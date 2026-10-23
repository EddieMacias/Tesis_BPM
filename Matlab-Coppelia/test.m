figure;

plot(trajectory01(:,1), trajectory01(:,2),LineWidth=1,Color="b");
hold on;

plot(realpos01(:,1), realpos01(:,2),LineWidth=1,Color="r");

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
title('Seguimiento de trayectoria - Dron 01');
legend('Posición comandada', 'Posición real', ...
    'Location', 'best');