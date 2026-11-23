%% Plotting function
function plot_dubins_results(X, U, T, R_avoid, c_avoid)
    N = length(U);
    tX = linspace(0,T,N+1);
    tU = linspace(0,T,N);
    
    % Motor commands
    figure(1)
    subplot(2,1,1)
    stairs(tU,U(1, :),'LineWidth',3); grid on;
    ylabel('$u$','Interpreter','latex')
    xlabel('time')

    subplot(2,1,2)
    stairs(tU,U(2, :),'LineWidth',3); grid on;
    ylabel('$u$','Interpreter','latex')
    xlabel('time')

    % Sampled drone locations
    figure(2)
    % plot3(X(1,:), X(2,:), X(3,:), 'LineWidth', 3); grid on; hold on;

    positions = X(1:2, :);
    rpy_rads = X(3, :);

    scatter(positions(1, :), positions(2, :)); hold on;

    p = nsidedpoly(1000, 'Center', c_avoid', 'Radius', R_avoid);
    plot(p, 'FaceColor', 'r')

    fontsize(18,"points")
end
