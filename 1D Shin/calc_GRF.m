function [ GRF, Fs, Fd ] = calc_GRF(x, dx, k, c )
%CALC_GRF calculated the ground reaction force
% Inputs:
%   - x:    position
%   - dx:   velocity
%   - k:    spring constant
%   - c:    damping coefficient

Fs = k*x;
Fd = c*dx;

GRF = Fs + Fd;
end

