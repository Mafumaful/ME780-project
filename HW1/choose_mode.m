if mode ==1
    if line == 1
        % straight line path, efficient mode
        kappa = 30;
        Q = diag([4.75,4.75,100]);
        R = diag([3,20]);
        E = 2.5*10^-10;
        % print 
        disp('straight line path, efficient mode');
    elseif line == 2
        % spline path, efficient mode
        kappa = 20;
        Q = diag([10,10,2]);
        R = diag([1,1]);
        E = 2.5*10^-10;
        disp('spline path, efficient mode');
    elseif line == 3
        % circle path, efficient mode
        kappa = 0.157;
        Q = diag([100,100,0]);
        R = diag([1,1]);
        E = 2.5*10^-10;
        disp('circle path, efficient mode');
    else
        disp('wrong line number');
    end
elseif mode == 2
    if line == 1
        % straight line path, sport mode
        kappa = 30;
        Q = diag([4.75,4.75,100]);
        R = diag([1,1]);
        E = 0;
        disp('straight line path, sport mode');
    elseif line == 2
        % spline path, sport mode
        kappa = 20;
        Q = diag([10,10,2]);
        R = diag([1,1]);
        E = 0;
        disp('spline path, sport mode');
    elseif line == 3
        % circle path, sport mode
        kappa = 0.157;
        Q = diag([100,100,0]);
        R = diag([1,1]);
        E = 0;
        disp('circle path, sport mode');
    else
        disp('wrong line number');
    end
elseif mode == 3
    % debut mode
    disp('debut mode');
    % straight line path, manual mode
    kappa = 30;
    Q = diag([100,100,0]);
    R = diag([0,0]);
    E = 0;
    disp('straight line path, manual mode');
    disp('wrong mode number');
end