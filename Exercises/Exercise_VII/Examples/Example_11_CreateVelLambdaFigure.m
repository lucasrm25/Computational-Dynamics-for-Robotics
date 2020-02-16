function Example_11_CreateVelLambdaFigure(FigNr, titleString, phi_vec, c_dot_PLUS, Lambda)
    % Show output:
    fig = figure(FigNr);
    clf
    set(fig,'Name',titleString)
    % Velocities:
    subplot(211)
    hold on
    grid on
    xlabel('Leg Angle at Impact $\varphi$ [deg]','interpreter','latex')
    ylabel('Post Impact Vel. [$l \dot{\varphi}_{tr}$]','interpreter','latex')
    plot(phi_vec,c_dot_PLUS(1,:),'m')
    plot(phi_vec,c_dot_PLUS(2,:),'r')
    plot(phi_vec,c_dot_PLUS(3,:),'c')
    plot(phi_vec,c_dot_PLUS(4,:),'b')
    L1 = legend('$\dot{x}_{tr}$',...
                '$\dot{y}_{tr}$',...
                '$\dot{x}_{le}$',...
                '$\dot{y}_{le}$');
    set(L1,'interpreter','latex','FontSize',11);
    axis([phi_vec(1), phi_vec(end), -2, 2])
    set(L1,'location','best');
    % Impulses:
    subplot(212)
    hold on
    grid on
    xlabel('Leg Angle at Impact $\varphi$ [deg]','interpreter','latex')
    ylabel('Impulse [$m l \dot{\varphi}_{tr}$]','interpreter','latex')
    plot(phi_vec,Lambda(1,:),'m')
    plot(phi_vec,Lambda(2,:),'r')
    plot(phi_vec,Lambda(3,:),'c')
    plot(phi_vec,Lambda(4,:),'b')
    L2 = legend('$\Lambda_{tr,x}$',...
                '$\Lambda_{tr,y}$',...
                '$\Lambda_{le,x}$',...
                '$\Lambda_{le,y}$');
    set(L2,'interpreter','latex','FontSize',11);
    axis([phi_vec(1), phi_vec(end), -2, 2])
    set(L1,'location','best');
end