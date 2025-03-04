function Tau = calculateTorque(paths, q, dq, ddq, dyn_model)
    last_index = size(paths{1}, 2);
    Tau = zeros(6, last_index);
    fprintf('Calculating torque...');
    for i = 1:last_index
         
        % Construct the array of path values for the current index
        path_values = zeros(1, 18);
        for j = 1:6
            path_values(1, j) = paths{j}(1, i);
            path_values(1, j + 6) = paths{j}(2, i);
            path_values(1, j + 12) = paths{j}(3, i);
        end
        
        Tau(1:6, i) = vpa(subs(dyn_model, ...
            [q, dq, ddq], path_values), 3);
    end
end
