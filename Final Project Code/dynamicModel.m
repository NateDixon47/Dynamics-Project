function Tau = dynamicModel(mass)
    disp('Solving for dynamic model...')
    syms a1 a2 a3 d1 d2 d3 real
    syms q1 q2 q3 q4 q5 q6 real
    syms dq1 dq2 dq3 dq4 dq5 dq6 real
    syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real

    q = [q1 q2 q3 q4 q5 q6];
    q_sym = [q1 q2 q3 q4 q5 q6]; % Define q_sym here
    dq_sym = [dq1 dq2 dq3 dq4 dq5 dq6];
    ddq_sym = [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6];

    m1 = 15;
    m2 = 10;
    m3 = 2;
    m4 = 5;
    m5 = 5;
    m6 = 2;

    mp = mass;
    g = 9.8;

    % m1 = 0;
    % m2 = 0;
    % m3 = 0;
    % m4 = 0;
    % m5 = 0;
    % m6 = 0;
    % 
    % mp = mass;
    % g = 9.8;

    % Calculate FK
    d = [d1 0 0 d2 0 d3];
    q = [q1 q2+pi/2 q3 q4 q5 q6];
    a = [a1 a2 a3 0 0 0];
    alpha = [pi/2 0 pi/2 -pi/2 pi/2 0];

    [T01, T02, T03, T04, T05, TF] = FK(q, d, a, alpha);

    % Calculate the Jacobian Matrix
    J1 = jacobian(T01(1:3,4), q1);
    J2 = jacobian(T02(1:3,4), [q1 q2]);
    J3 = jacobian(T03(1:3,4), [q1 q2 q3]);
    J4 = jacobian(T04(1:3,4), [q1 q2 q3 q4]);
    J5 = jacobian(T05(1:3,4), [q1 q2 q3 q4 q5]);
    J6 = jacobian(TF(1:3,4), [q1 q2 q3 q4 q5 q6]);

    v_m1 = J1 * dq1;
    v_m2 = J2 * [dq1; dq2];
    v_m3 = J3 * [dq1; dq2; dq3];
    v_m4 = J4 * [dq1; dq2; dq3; dq4];
    v_m5 = J5 * [dq1; dq2; dq3; dq4; dq5];
    v_m6 = J6 * [dq1; dq2; dq3; dq4; dq5; dq6];

    % Calculate the kinetic and potential energy of the system
    fprintf('Calculating Kinetic and Potential Energy...\n');
    K1 = simplify(subs(0.5 * m1 * (v_m1.' * v_m1), [a1 d1], [a1/2 d1/2]));
    K2 = subs(0.5 * m2 * (v_m2.' * v_m2), a2, a2/2);
    K3 = subs(0.5 * m3 * (v_m3.' * v_m3), [a3 d2], [a3/2 d2/2]);
    K4 = subs(0.5 * m4 * (v_m4.' * v_m4), [a3 d2], [a3/2 d2/2]);
    K5 = subs(0.5 * m5 * (v_m5.' * v_m5), [a3 d2], [a3/2 d2/2]);
    K6 = subs(0.5 * m6 * (v_m6.' * v_m6), d3, d3/2);
    K_p = 0.5 * mp * (v_m6.' * v_m6);

    P1 = subs(m1 * g * T01(3,4), [a1 d1], [a1/2 d1/2]);
    P2 = subs(m2 * g * T02(3,4), a2, a2/2);
    P4 = subs(m4 * g * T04(3,4), [a3 d2], [a3/2 d2/2]);
    P6 = subs(m6 * g * TF(3,4), d3, d3/2);
    P_payload = mp * g * TF(3,4);

    fprintf('Calculating Lagrange...\n');
    K = K1 + K2 + K3 + K4 + K5 + K6 + K_p;
    P = P1 + P2 + P4 + P6 + P_payload;

    % Calculate Lagrange
    L = K - P;

    % Initialize empty arrays for A and B
    A = sym(zeros(6, 1));
    B = sym(zeros(6, 1));
    Tau_l = sym(zeros(6, 1));
    % Calculate A, B, and Tau_l in a loop
    for i = 1:6
        A(i) = calculateLagrangeTerm(L, q_sym, dq_sym, ddq_sym, i);
        B(i) = diff(L, q_sym(i)); % Use q_sym here
        Tau_l(i) = A(i) - B(i);
    end
    
    % Substitute the DH parameters into the tau expressions
    Tau = vpa(subs(Tau_l, [a1, a2, a3, d1, d2, d3], [50, 200, 35, 200, 250, 80]), 3);
end


