%% File: wind_model_quad.m
% Time-varying lateral wind acceleration (m/s^2)
function aw = wind_model_quad(t, par)
    aw = par.wind_amp * sin(par.wind_freq * t);
end