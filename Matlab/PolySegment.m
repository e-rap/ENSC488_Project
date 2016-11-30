function [ pos, vel, accel ] = PolySegment( p0, pf, v0, vf)
%POLYSEGMENT Summary of this function goes here
%   Detailed explanation goes here

syms b0 b1 b2 b3 t

pos(t) = b0 + b1*t + b2*t^2 + b3*t^3;
vel(t) = diff(pos,t);

eqn1 = pos(0) == p0;
eqn2 = pos(1) == pf;
eqn3 = vel(0) == v0;
eqn4 = vel(1) == vf;

[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4],[b0, b1, b2, b3]);
X = linsolve(A,B);

pos = X(1) + X(2)*t + X(3)*t^2 + X(4)*t^3;
vel = diff(pos,t);
accel = diff(vel,t);
end

