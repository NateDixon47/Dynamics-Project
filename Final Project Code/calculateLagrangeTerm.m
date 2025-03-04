function A = calculateLagrangeTerm(L, q, dq, ddq, joint_num)
    % Calculate the Lagrange term A for a specific joint in the dynamic model

    % Define symbolic time-dependent variables for joint angles and their derivatives
    syms th1(t) th2(t) th3(t) th4(t) th5(t) th6(t)
    th = [th1(t) th2(t) th3(t) th4(t) th5(t) th6(t)];
    dth = diff(th, t);

    % Compute the partial derivative of the Lagrangian with respect to the generalized velocity
    partialL_dq = diff(L, dq(joint_num));
    
    % Substitute the generalized coordinates and velocities with time-dependent variables
    partialL_dq_subs = subs(partialL_dq, [q, dq], [th, dth]);
    
    % Compute the total time derivative of the substituted expression
    d_partialL_dq_subs = diff(partialL_dq_subs, t);
    
    % Substitute back the time-dependent variables with generalized coordinates, velocities, and accelerations
    A = subs(d_partialL_dq_subs, ...
        [th, dth, diff(th, t, 2)], ...
        [q, dq, ddq]);
end
