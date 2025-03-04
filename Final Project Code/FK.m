function [T01, T02, T03, T04, T05, TF] = FK(q, d, a, alpha)

T01 = DH(alpha(1), a(1), d(1), q(1));
T12 = DH(alpha(2), a(2), d(2), q(2));
T23 = DH(alpha(3), a(3), d(3), q(3));
T34 = DH(alpha(4), a(4), d(4), q(4));
T45 = DH(alpha(5), a(5), d(5), q(5));
T5f = DH(alpha(6), a(6), d(6), q(6));

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
TF = T05 * T5f;
end