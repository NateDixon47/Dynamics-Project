function qt = quinticTrajectory(q0, v0, a0, qf, vf, af, t)
    % qt will store positions, velocities, and accelerations
    qt = zeros(3, length(t), length(q0));

    % Loop through each joint
    for i = 1:length(q0)
        % Extract boundary conditions for the current joint
        q0_i = q0(i);
        v0_i = v0(i);
        a0_i = a0(i);
        qf_i = qf(i);
        vf_i = vf(i);
        af_i = af(i);

        % Boundary conditions vector
        b = [q0_i; v0_i; a0_i; qf_i; vf_i; af_i];

        % Boundary conditions matrix
        t0 = t(1);
        tf = t(end);
        M = [1 t0 t0^2 t0^3 t0^4 t0^5;
             0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
             0 0 2 6*t0 12*t0^2 20*t0^3;
             1 tf tf^2 tf^3 tf^4 tf^5;
             0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
             0 0 2 6*tf 12*tf^2 20*tf^3];

        % Solve for coefficients
        a = M\b;

        % Calculate positions, velocities, and accelerations for each time step
        c = ones(size(t));
        qt(1,:,i) = a(1).*c + a(2).*t + a(3).*t.^2 + a(4).*t.^3 + a(5).*t.^4 + a(6).*t.^5; % Position
        qt(2,:,i) = a(2).*c + 2*a(3).*t + 3*a(4).*t.^2 + 4*a(5).*t.^3 + 5*a(6).*t.^4; % Velocity
        qt(3,:,i) = 2*a(3).*c + 6*a(4).*t + 12*a(5).*t.^2 + 20*a(6).*t.^3; % Acceleration
    end
end
