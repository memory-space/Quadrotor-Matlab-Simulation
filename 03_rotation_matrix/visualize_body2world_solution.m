function visualize_body2world_solution()
    %% VISUALIZE_BODY2WORLD - Interactive visualization with target matching
    %
    % This function helps to verify their body2world_xyz function
    % by showing a target orientation and checking if their input matches it.
    %
    % Features:
    %   - Left plot: Initial body frame (identity orientation)
    %   - Right plot: Target pose (red dotted, thin) + result (solid RGB, thick)
    %   - Target angles displayed on screen
    %   - Enters angles to match the target
    %   - Visual feedback: solid lines overlap dotted lines = SUCCESS!
    %
    % Usage:
    %   visualize_body2world_xyz()
    %   1. See target angles displayed
    %   2. Enter those angles in input boxes
    %   3. Click "Apply Rotation"
    %   4. Check if solid body frame matches dotted target!
    
    %% Generate random target angles (or set specific values for testing)
    target_roll = 30;   % degrees
    target_pitch = 45;  % degrees
    target_yaw = 60;    % degrees
    
    %% Calculate target rotation matrix
    target_R = calculate_target_rotation(target_roll, target_pitch, target_yaw);
    
    %% Create figure
    fig = figure('Name', 'Body-to-World Transformation Visualizer', ...
                 'Position', [50, 50, 1400, 700], ...
                 'Color', [0.95 0.95 0.95]);
    
    %% Left subplot - Initial state
    ax_initial = subplot(1, 2, 1);
    set(ax_initial, 'Position', [0.05, 0.25, 0.4, 0.65]);
    hold(ax_initial, 'on'); 
    grid(ax_initial, 'on'); 
    axis(ax_initial, 'equal'); 
    view(ax_initial, 3);
    xlabel(ax_initial, 'X', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax_initial, 'Y', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel(ax_initial, 'Z', 'FontSize', 12, 'FontWeight', 'bold');
    title(ax_initial, 'Initial: Body Frame = World Frame', 'FontSize', 14, 'FontWeight', 'bold');
    xlim(ax_initial, [-1.5, 1.5]);
    ylim(ax_initial, [-1.5, 1.5]);
    zlim(ax_initial, [-1.5, 1.5]);
    
    %% Right subplot - Target + Result
    ax_target = subplot(1, 2, 2);
    set(ax_target, 'Position', [0.52, 0.25, 0.4, 0.65]);
    hold(ax_target, 'on');
    grid(ax_target, 'on');
    axis(ax_target, 'equal');
    view(ax_target, 3);
    xlabel(ax_target, 'X', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax_target, 'Y', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel(ax_target, 'Z', 'FontSize', 12, 'FontWeight', 'bold');
    
    %% Title with target angles integrated
    title_handle = title(ax_target, ...
                        {'\bf Target (dotted) vs Your Result (solid)', ...
                         sprintf('Target: Roll=%.0f°, Pitch=%.0f°, Yaw=%.0f°', ...
                                 target_roll, target_pitch, target_yaw)}, ...
                        'FontSize', 12);
    xlim(ax_target, [-1.5, 1.5]);
    ylim(ax_target, [-1.5, 1.5]);
    zlim(ax_target, [-1.5, 1.5]);
    
    %% Input controls (moved up since target display removed)
    uicontrol('Style', 'text', 'Position', [50, 150, 150, 25], ...
              'String', 'Roll (φ) [deg]:', ...
              'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.95 0.95 0.95], ...
              'HorizontalAlignment', 'left');
    roll_input = uicontrol('Style', 'edit', ...
                           'Position', [210, 150, 100, 25], ...
                           'String', '0', ...
                           'FontSize', 11);
    
    uicontrol('Style', 'text', 'Position', [50, 115, 150, 25], ...
              'String', 'Pitch (θ) [deg]:', ...
              'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.95 0.95 0.95], ...
              'HorizontalAlignment', 'left');
    pitch_input = uicontrol('Style', 'edit', ...
                            'Position', [210, 115, 100, 25], ...
                            'String', '0', ...
                            'FontSize', 11);
    
    uicontrol('Style', 'text', 'Position', [50, 80, 150, 25], ...
              'String', 'Yaw (ψ) [deg]:', ...
              'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.95 0.95 0.95], ...
              'HorizontalAlignment', 'left');
    yaw_input = uicontrol('Style', 'edit', ...
                          'Position', [210, 80, 100, 25], ...
                          'String', '0', ...
                          'FontSize', 11);
    
    %% Apply button
    uicontrol('Style', 'pushbutton', ...
              'Position', [330, 80, 120, 50], ...
              'String', 'Apply Rotation', ...
              'FontSize', 12, ...
              'FontWeight', 'bold', ...
              'Callback', @apply_rotation, ...
              'BackgroundColor', [0.3 0.7 0.3], ...
              'ForegroundColor', [1 1 1]);
    
    %% Animate checkbox
    animate_check = uicontrol('Style', 'checkbox', ...
                              'Position', [330, 140, 150, 25], ...
                              'String', 'Animate Transition', ...
                              'FontSize', 10, ...
                              'Value', 1, ...
                              'BackgroundColor', [0.95 0.95 0.95]);
    
    %% New Target button
    uicontrol('Style', 'pushbutton', ...
              'Position', [470, 80, 150, 50], ...
              'String', 'New Target', ...
              'FontSize', 11, ...
              'FontWeight', 'bold', ...
              'Callback', @generate_new_target, ...
              'BackgroundColor', [0.3 0.5 0.9], ...
              'ForegroundColor', [1 1 1]);
    
    %% Gimbal Lock test button
    uicontrol('Style', 'pushbutton', ...
              'Position', [470, 140, 150, 25], ...
              'String', 'Test Gimbal Lock', ...
              'FontSize', 10, ...
              'Callback', @test_gimbal_lock, ...
              'BackgroundColor', [0.9 0.5 0.2], ...
              'ForegroundColor', [1 1 1]);
    
    %% Info panel
    info_panel = uicontrol('Style', 'text', ...
                           'Position', [650, 15, 600, 140], ...
                           'String', '', ...
                           'FontSize', 9, ...
                           'HorizontalAlignment', 'left', ...
                           'BackgroundColor', [1 1 1], ...
                           'FontName', 'Courier New');
    
    %% Match status indicator (simple text, no box)
    match_status = uicontrol('Style', 'text', ...
                             'Position', [50, 20, 570, 40], ...
                             'String', '', ...
                             'FontSize', 13, ...
                             'FontWeight', 'bold', ...
                             'BackgroundColor', [0.95 0.95 0.95], ...
                             'ForegroundColor', [0.5 0.5 0.5], ...
                             'HorizontalAlignment', 'center');
    
    %% Draw initial frames
    draw_initial_frame(ax_initial);
    draw_target_frame(ax_target, target_R, target_roll, target_pitch, target_yaw);
    update_info_panel(eye(3), 0, 0, 0, target_R, target_roll, target_pitch, target_yaw);
    
    %% Internal functions
    function apply_rotation(~, ~)
        % Get input values
        roll_deg = str2double(get(roll_input, 'String'));
        pitch_deg = str2double(get(pitch_input, 'String'));
        yaw_deg = str2double(get(yaw_input, 'String'));
        
        % Validate
        if isnan(roll_deg) || isnan(pitch_deg) || isnan(yaw_deg)
            errordlg('Please enter valid numbers!', 'Input Error');
            return;
        end
        
        % Convert to radians
        phi = deg2rad(roll_deg);
        theta = deg2rad(pitch_deg);
        psi = deg2rad(yaw_deg);
        
        try
            % Calculate the final rotation matrix first
            R_s = body2world_xyz_solution(phi, theta, psi);
            
            if get(animate_check, 'Value')
                % Call updated animation function with the final matrix
                animate_transformation_from_R(R_s);
            else
                draw_s_and_target(ax_target, R_s, target_R, ...
                                       roll_deg, pitch_deg, yaw_deg, ...
                                       target_roll, target_pitch, target_yaw);
                update_info_panel(R_s, roll_deg, pitch_deg, yaw_deg, ...
                                 target_R, target_roll, target_pitch, target_yaw);
                check_match(R_s, target_R);
            end
        catch ME
            errordlg(['Error: ' ME.message], 'Function Error');
        end
    end
    
    function animate_transformation_from_R(R_final)
        % Decompose R into Euler angles (ZYX sequence)
        % R = Rz(psi) * Ry(theta) * Rx(phi)
        theta_ext = asin(-R_final(3,1));
        phi_ext   = atan2(R_final(3,2), R_final(3,3));
        psi_ext   = atan2(R_final(2,1), R_final(1,1));
        
        % Target angles in degrees
        y_target = rad2deg(psi_ext);
        p_target = rad2deg(theta_ext);
        r_target = rad2deg(phi_ext);

        n_steps = 10; % Steps per axis for smooth motion
        pause_time = 0.01;
        
        % Stage 1: Animate Yaw (Z-axis)
        for i = 1:n_steps
            t = i / n_steps;
            curr_y = y_target * t;
            render_frame(0, 0, curr_y, false);
            pause(pause_time);
        end
        
        % Stage 2: Animate Pitch (Y-axis)
        for i = 1:n_steps
            t = i / n_steps;
            curr_p = p_target * t;
            render_frame(0, curr_p, y_target, false);
            pause(pause_time);
        end
        
        % Stage 3: Animate Roll (X-axis)
        for i = 1:n_steps
            t = i / n_steps;
            curr_r = r_target * t;
            is_last = (i == n_steps);
            render_frame(curr_r, p_target, y_target, is_last);
            pause(pause_time);
        end
    end

    % Helper function to handle rendering during animation
    function render_frame(r, p, y, check_match_flag)
        phi = deg2rad(r);
        theta = deg2rad(p);
        psi = deg2rad(y);
        
        % Re-calculate intermediate matrix for visualization
        R_step = body2world_xyz_solution(phi, theta, psi);
        
        draw_s_and_target(ax_target, R_step, target_R, ...
                               r, p, y, ...
                               target_roll, target_pitch, target_yaw);
        
        if check_match_flag
            update_info_panel(R_step, r, p, y, ...
                             target_R, target_roll, target_pitch, target_yaw);
            check_match(R_step, target_R);
        end
    end
    
    function generate_new_target(~, ~)
        % Generate new random target (avoiding gimbal lock)
        target_roll = randi([-180, 180]);
        target_pitch = randi([-80, 80]);  % Avoid ±90 gimbal lock
        target_yaw = randi([-180, 180]);
        
        % Update target rotation matrix
        target_R = calculate_target_rotation(target_roll, target_pitch, target_yaw);
        
        % Update title with new target angles
        title(ax_target, ...
              {'\bf Target (dotted) vs Your Result (solid)', ...
               sprintf('Target: Roll=%.0f°, Pitch=%.0f°, Yaw=%.0f°', ...
                       target_roll, target_pitch, target_yaw)}, ...
              'FontSize', 12);
        
        % Reset inputs
        set(roll_input, 'String', '0');
        set(pitch_input, 'String', '0');
        set(yaw_input, 'String', '0');
        
        % Redraw target
        draw_target_frame(ax_target, target_R, target_roll, target_pitch, target_yaw);
        update_info_panel(eye(3), 0, 0, 0, target_R, target_roll, target_pitch, target_yaw);
        set(match_status, 'String', '', 'ForegroundColor', [0.5 0.5 0.5]);
    end
    
    function test_gimbal_lock(~, ~)
        % Set gimbal lock target
        target_roll = 45;
        target_pitch = 90;
        target_yaw = 30;
        target_R = calculate_target_rotation(target_roll, target_pitch, target_yaw);
        
        % Update title with gimbal lock warning
        title(ax_target, ...
              {'\bf Target (dotted) vs Your Result (solid)', ...
               sprintf('Target: Roll=%.0f°, Pitch=%.0f°, Yaw=%.0f° [GIMBAL LOCK!]', ...
                       target_roll, target_pitch, target_yaw)}, ...
              'FontSize', 12, 'Color', [0.8 0 0]);
        
        % Reset inputs
        set(roll_input, 'String', '0');
        set(pitch_input, 'String', '0');
        set(yaw_input, 'String', '0');
        
        % Redraw target frame
        draw_target_frame(ax_target, target_R, target_roll, target_pitch, target_yaw);
        
        % Update info panel with reset state
        update_info_panel(eye(3), 0, 0, 0, target_R, target_roll, target_pitch, target_yaw);
        
        % Clear match status
        set(match_status, 'String', '', 'ForegroundColor', [0.5 0.5 0.5]);
        
        msgbox(['Gimbal Lock Test!' newline newline ...
               'Try entering the target angles.' newline ...
               'Then try different Roll/Yaw combinations.' newline ...
               'Notice how they produce similar results!'], ...
               'Gimbal Lock', 'warn');
    end
    
    function check_match(R_s, R_target)
        % Calculate error
        error_norm = norm(R_s - R_target, 'fro');
        
        if error_norm < 0.01
            set(match_status, 'String', '✓ Perfect Match!', ...
                'ForegroundColor', [0 0.6 0]);
        elseif error_norm < 0.1
            set(match_status, 'String', '~ Close (small error)', ...
                'ForegroundColor', [0.8 0.5 0]);
        else
            set(match_status, 'String', '✗ No match - check your angles', ...
                'ForegroundColor', [0.8 0 0]);
        end
    end
    
    function update_info_panel(R_s, r, p, y, R_target, r_t, p_t, y_t)
        error_norm = norm(R_s - R_target, 'fro');
        
        % Format matrices side by side
        info_str = sprintf(['YOUR MATRIX:              TARGET MATRIX:\n' ...
                           '[%6.3f %6.3f %6.3f]   [%6.3f %6.3f %6.3f]\n' ...
                           '[%6.3f %6.3f %6.3f]   [%6.3f %6.3f %6.3f]\n' ...
                           '[%6.3f %6.3f %6.3f]   [%6.3f %6.3f %6.3f]\n\n' ...
                           'Your: R=%.1f° P=%.1f° Y=%.1f°\n' ...
                           'Target: R=%.1f° P=%.1f° Y=%.1f°\n\n' ...
                           'Error: %.6f\n'], ...
                           R_s(1,1), R_s(1,2), R_s(1,3), R_target(1,1), R_target(1,2), R_target(1,3), ...
                           R_s(2,1), R_s(2,2), R_s(2,3), R_target(2,1), R_target(2,2), R_target(2,3), ...
                           R_s(3,1), R_s(3,2), R_s(3,3), R_target(3,1), R_target(3,2), R_target(3,3), ...
                           r, p, y, ...
                           r_t, p_t, y_t, error_norm);
        
        set(info_panel, 'String', info_str);
    end
end

function R = calculate_target_rotation(roll_deg, pitch_deg, yaw_deg)
    % Calculate correct rotation matrix for target
    phi = deg2rad(roll_deg);
    theta = deg2rad(pitch_deg);
    psi = deg2rad(yaw_deg);
    
    R_x = [1,        0,         0;
           0,  cos(phi), -sin(phi);
           0,  sin(phi),  cos(phi)];
    
    R_y = [ cos(theta), 0, sin(theta);
                  0,    1,      0;
           -sin(theta), 0, cos(theta)];
    
    R_z = [cos(psi), -sin(psi), 0;
           sin(psi),  cos(psi), 0;
               0,         0,    1];
    
    R = R_z * R_y * R_x;  % Correct ZYX body-to-world
end

function draw_initial_frame(ax)
    cla(ax);
    hold(ax, 'on');
    scale = 1.0;
    
    % World frame (gray) - NED convention (Z points DOWN)
    quiver3(ax, 0, 0, 0, scale, 0, 0, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, 0, scale, 0, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, 0, 0, -scale, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);  % Z down
    
    % Body frame (same as world initially) - NED convention
    quiver3(ax, 0, 0, 0, scale, 0, 0, 'r', 'LineWidth', 4);
    quiver3(ax, 0, 0, 0, 0, scale, 0, 'g', 'LineWidth', 4);
    quiver3(ax, 0, 0, 0, 0, 0, -scale, 'b', 'LineWidth', 4);  % Z down
    
    text(ax, scale*1.2, 0, 0, 'X_b', 'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(ax, 0, scale*1.2, 0, 'Y_b', 'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    text(ax, 0, 0, -scale*1.2, 'Z_b (down)', 'FontSize', 12, 'Color', 'b', 'FontWeight', 'bold');
end

function draw_target_frame(ax, R_target, r_t, p_t, y_t)
    cla(ax);
    hold(ax, 'on');
    scale = 1.0;
    
    % World frame (gray) - NED convention
    quiver3(ax, 0, 0, 0, scale, 0, 0, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, 0, scale, 0, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, 0, 0, -scale, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);  % Z down
    
    % Target frame (RED DOTTED, THIN) - Apply rotation to NED initial frame
    x_t = R_target * [scale; 0; 0];
    y_t = R_target * [0; scale; 0];
    z_t = R_target * [0; 0; -scale];  % Start with Z pointing down
    
    quiver3(ax, 0, 0, 0, x_t(1), x_t(2), x_t(3), ...
            'Color', [1 0 0], 'LineStyle', ':', 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, y_t(1), y_t(2), y_t(3), ...
            'Color', [1 0 0], 'LineStyle', ':', 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, z_t(1), z_t(2), z_t(3), ...
            'Color', [1 0 0], 'LineStyle', ':', 'LineWidth', 2);
    
    text(ax, x_t(1)*1.2, x_t(2)*1.2, x_t(3)*1.2, 'Target', ...
         'FontSize', 10, 'Color', [1 0 0], 'FontWeight', 'bold');
end

function draw_s_and_target(ax, R_s, R_target, r, p, y, r_t, p_t, y_t)
    cla(ax);
    hold(ax, 'on');
    scale = 1.0;
    
    % World frame (gray) - NED convention
    quiver3(ax, 0, 0, 0, scale, 0, 0, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, 0, scale, 0, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, 0, 0, -scale, 'Color', [0.7 0.7 0.7], 'LineWidth', 2);  % Z down
    
    % Target frame (RED DOTTED, THIN) - always visible
    x_t = R_target * [scale; 0; 0];
    y_t = R_target * [0; scale; 0];
    z_t = R_target * [0; 0; -scale];  % Start with Z pointing down
    
    quiver3(ax, 0, 0, 0, x_t(1), x_t(2), x_t(3), ...
            'Color', [1 0 0], 'LineStyle', ':', 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, y_t(1), y_t(2), y_t(3), ...
            'Color', [1 0 0], 'LineStyle', ':', 'LineWidth', 2);
    quiver3(ax, 0, 0, 0, z_t(1), z_t(2), z_t(3), ...
            'Color', [1 0 0], 'LineStyle', ':', 'LineWidth', 2);
    
    % S frame (RGB SOLID, THICK)
    x_s = R_s * [scale; 0; 0];
    y_s = R_s * [0; scale; 0];
    z_s = R_s * [0; 0; -scale];  % Start with Z pointing down
    
    quiver3(ax, 0, 0, 0, x_s(1), x_s(2), x_s(3), 'r', 'LineWidth', 4);
    quiver3(ax, 0, 0, 0, y_s(1), y_s(2), y_s(3), 'g', 'LineWidth', 4);
    quiver3(ax, 0, 0, 0, z_s(1), z_s(2), z_s(3), 'b', 'LineWidth', 4);
    
    text(ax, x_s(1)*1.2, x_s(2)*1.2, x_s(3)*1.2, 'X_b', ...
         'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    text(ax, y_s(1)*1.2, y_s(2)*1.2, y_s(3)*1.2, 'Y_b', ...
         'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    text(ax, z_s(1)*1.2, z_s(2)*1.2, z_s(3)*1.2, 'Z_b', ...
         'FontSize', 12, 'Color', 'b', 'FontWeight', 'bold');
end