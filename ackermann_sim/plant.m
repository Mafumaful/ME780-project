function [sys, x0, str, ts, simStateCompliance] = plant(t, x, u, flag, param)

    switch flag

        case 0
            [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes;

        case 1
            sys = mdlDerivatives(t, x, u, param);

        case 2
            sys = mdlUpdate(t, x, u);

        case 3
            sys = mdlOutputs(t, x, u);

        case 4
            sys = mdlGetTimeOfNextVarHit(t, x, u);

        case 9
            sys = mdlTerminate(t, x, u);

        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

    end

end

function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes
    import casadi.*

    sizes = simsizes;

    sizes.NumContStates = 2;
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 2;
    sizes.NumInputs = 1;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 0; % at least one sample time is needed

    sys = simsizes(sizes);

    x0 = [0 0];

    str = [];

    ts = [];

    simStateCompliance = 'UnknownSimState';

end

function sys = mdlDerivatives(t, x, u, param)

    k = pa.k;
    m = pa.m;

    x1 = x(1);
    x2 = x(2);

    x1dot = x2;
    x2dot = -k / m * x1 ^ 3 + 1 / m * u;

    sys = [x1dot; x2dot];

end

function sys = mdlUpdate(t, x, u)

    sys = [];

end

function sys = mdlOutputs(t, x, u)

    sys = x;

end

function sys = mdlGetTimeOfNextVarHit(t, x, u)

    sampleTime = 1; %  Example, set the next hit to be one second later.
    sys = t + sampleTime;

end

function sys = mdlTerminate(t, x, u)

    sys = [];
end
