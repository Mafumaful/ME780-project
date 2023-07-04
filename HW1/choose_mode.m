if cmode ==1
    if cline == 1
        % straight cline path, efficient mode
        kappa = 30;
        Q = diag([4.75,4.75,100]);
        R = diag([3,20]);
        E = 2.5*10^-10;
        % print 
        disp('straight cline path, efficient mode');
    elseif cline == 2
        % spcline path, efficient mode
        kappa = 20;
        Q = diag([10,10,2]);
        R = diag([2,2]);
        E = 2.5*10^-10;
        disp('spcline path, efficient mode');
    elseif cline == 3
        % circle path, efficient mode
        kappa = 10;
        Q = diag([100,100,0]);
        R = diag([5,5]);
        E = 2.5*10^-10;
        disp('circle path, efficient mode');
    else
        disp('wrong cline number');
    end
elseif cmode == 2
    if cline == 1
        % straight cline path, sport mode
        kappa = 30;
        Q = diag([4.75,4.75,100]);
        R = diag([1,1]);
        E = 0;
        disp('straight cline path, sport mode');
    elseif cline == 2
        % spcline path, sport mode
        kappa = 20;
        Q = diag([10,10,2]);
        R = diag([1,1]);
        E = 0;
        disp('spcline path, sport mode');
    elseif cline == 3
        % circle path, sport mode
        kappa = 10;
        Q = diag([100,100,0]);
        R = diag([1,1]);
        E = 0;
        disp('circle path, sport mode');
    else
        disp('wrong cline number');
    end
elseif cmode == 3
    % debut mode
    disp('debut mode');
    % straight cline path, manual mode
    kappa = 30;
    Q = diag([100,100,0]);
    R = diag([0,0]);
    E = 0;
    disp('straight cline path, manual mode');
    disp('wrong mode number');
end
