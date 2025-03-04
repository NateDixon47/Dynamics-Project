function z = extract_z_axis(T)
    R = T(1:3,1:3);
    z = R * [0;0;1];
end