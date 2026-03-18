function animate(t, x, param)
    % Quadrotor arm length (Center to Motor: 46mm = 0.046m)
    L = 8*param.arm_length_from_center; 
    
    % Figure setting
    fig = figure('Name', 'Quadrotor 3D Animation', 'Color', 'w');
    xlabel('North (m)'); 
    ylabel('East (m)'); 
    zlabel('Down (m)');
    view(3); 
    axis equal; 
    grid on; 
    hold on;
    
    set(gca, 'ZDir','reverse'); 
    
    % Automatic setting of axis range according to simulation range
    axis([min(x(:,1))-0.5, max(x(:,1))+0.5, ...
          min(x(:,2))-0.5, max(x(:,2))+0.5, ...
          min(x(:,3))-1.0,             0.5]);

    % Line Objects (X-Form) for quadrotor Airframe Configuration
    % arm13: Number 1 (FR) - Number 3 (BL) connection / arm24: Number 2 (FL) - Number 4 (BR) connection
    arm13 = plot3(0,0,0, 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
    arm24 = plot3(0,0,0, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');

    x_arrow = quiver3(0, 0, 0, 1, 0, 0, 'Color', '#00AA00', 'LineWidth', 2.5, ...
                      'MaxHeadSize', 2, 'AutoScale', 'off');
    
    % Line for Trace Display
    trace = plot3(x(:,1), x(:,2), x(:,3), 'k:', 'LineWidth', 0.5);

    % Animated Loop
    skip = 10; % If it's too slow, grow the numbers (1 frame per 10 steps)
    for i = 1:skip:length(t)
        p = x(i, 1:3)'; 
        phi = x(i, 7); 
        theta = x(i, 8); 
        psi = x(i, 9);
        
        R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
             cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
             -sin(theta),         sin(phi)*cos(theta),                        cos(phi)*cos(theta)];
        
        L_c = L * cosd(45);
        p1 = R * [ L_c; -L_c; 0] + p;
        p2 = R * [ L_c;  L_c; 0] + p;
        p3 = R * [-L_c;  L_c; 0] + p;
        p4 = R * [-L_c; -L_c; 0] + p;
        
        set(arm13, 'XData', [p1(1) p3(1)], 'YData', [p1(2) p3(2)], 'ZData', [p1(3) p3(3)]);
        set(arm24, 'XData', [p2(1) p4(1)], 'YData', [p2(2) p4(2)], 'ZData', [p2(3) p4(3)]);
        
        arrow_vec = R * [L * 3.0; 0; 0];
        
        set(x_arrow, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                     'UData', arrow_vec(1), 'VData', arrow_vec(2), 'WData', arrow_vec(3));
        
        title(sprintf('Time: %.2f s / Altitude: %.2f m', t(i), -p(3)));
        
        drawnow;
        if ~ishandle(fig), break; end
    end
end

% Author: Lim Minseok (minseoklim@postech.ac.kr)