function [w1, w2, w3, w4] = drone_input(Kt, Kd, l);
syms w1 w2 w3 w4 Tx Ty Tz F;
eqn1 = 0 == (w1^2 - w3^2)*Kt*l - Tx;
eqn2 = 0 == (w2^2 - w4^2)*Kt*l - Ty;
eqn3 = 0 == (w2^2 + w4^2 - w1^2 - w3^2)*Kd - Tz;
eqn4 = 0 == Kt*(w1^2 + w2^2 + w3^2 + w4^2) - F;

assume(w1 >= 0);
assume(w2 >= 0);
assume(w3 >= 0);
assume(w4 >= 0);
assume(F >= 0);

S = solve([eqn1, eqn2, eqn3, eqn4], [w1, w2, w3, w4], 'ReturnConditions', true);
w1 = matlabFunction(Kt*vpa(S.w1, 4)^2);
w2 = matlabFunction(Kt*vpa(S.w2, 4)^2);
w3 = matlabFunction(Kt*vpa(S.w3, 4)^2);
w4 = matlabFunction(Kt*vpa(S.w4, 4)^2);
end