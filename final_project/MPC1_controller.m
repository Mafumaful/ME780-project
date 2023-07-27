function [sys,x0,str,ts] = MPC1_controller(t,x,u,flag)
    switch flag
      case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
      case 2
        sys=mdlUpdate(t,x,u);
      case 3
        sys=mdlOutputs(t,x,u);
      case {1,4,9}
        sys=[];
      otherwise
        error.(['Unhandled flag=',num2str(flag)]);
    
    end
    
    function [sys,x0,str,ts]=mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 4;
    sizes.NumOutputs     = 1;
    sizes.NumInputs      = 5;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;   % at least one sample time is needed
    sys = simsizes(sizes);
    x0  = [0.0001,0.0001,0.0001,0.0001]';
    str = [];
    ts  = [0.01 0];
    global U
    U=0;
    
    function sys=mdlUpdate(~,x,~)
    
    sys = x;
    
    function sys=mdlOutputs(~,~,u)
    global A1 B1 u_piao U kesi
    Nx=4;
    Nu=1;
    Np=100; % predict horizon
    Nc=100; % control horizon
    Row=10;
    T=0.02; % predict step time

    m=1807.2;
    a=1.18;
    b=1.77;
    Iz=2687.1;
    k1=-110730;
    k2=-80188;
    
    vx=u(5);
    kesi=zeros(Nx+Nu,1);
    kesi(1)=u(1);
    kesi(2)=u(2);
    kesi(3)=u(3);
    kesi(4)=u(4);
    kesi(5)=U;
    q=[100,0,0,0;
       0,1,0,0;
       0,0,1,0;
       0,0,0,1]; % state cost function
    Q=kron(eye(Np),q);
    R=0.1*eye(Nc*Nu); % control cost function
    A2=[0,vx,1,0;
       0,0,0,1;
       0,0,(k1+k2)/m/vx,(a*k1-b*k2)/m/vx-vx;
       0,0,(a*k1-b*k2)/Iz/vx,(a^2*k1+b^2*k2)/Iz/vx];
    B2=[0;0;-k1/m;-a*k1/Iz];
    A1=A2*T+eye(Nx);
    B1=B2*T;
    A_cell=cell(2,2);
    A_cell{1,1}=A1;
    A_cell{1,2}=B1;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    A=cell2mat(A_cell);
    B_cell=cell(2,1);
    B_cell{1,1}=B1;
    B_cell{2,1}=eye(Nu);
    B=cell2mat(B_cell);
    C_cell=cell(1,2);
    C_cell{1,1}=eye(Nx);
    C_cell{1,2}=zeros(Nx,Nu);
    C=cell2mat(C_cell);
    
    PHI_cell=cell(Np,1);
    for i=1:1:Np
        PHI_cell{i,1}=C*A^i;
    end
    PHI=cell2mat(PHI_cell);
    THETA_cell=cell(Np,Nc);
    for i=1:1:Np
        for j=1:1:Nc
            if  i>=j
                THETA_cell{i,j}=C*A^(i-j)*B;
            else
                THETA_cell{i,j}=zeros(Nx,Nu);
            end
        end
    end
    THETA=cell2mat(THETA_cell);
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nc*Nu,1);
    H_cell{2,1}=zeros(1,Nc*Nu);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);
    error=PHI*kesi;
    f_cell=cell(2,1);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
    f=cell2mat(f_cell);
    A_t=zeros(Nc,Nc);
    for i=1:1:Nc
        for j=1:1:Nc
            if i>=j
                A_t(i,j)=1;
            else
                A_t(i,j)=0;
            end
        end
    end
    A_I=kron(A_t,eye(Nu));
    Ut=kron(ones(Nc,1),U);
    umax=0.44;
    umin=-0.44;
    umax_dt=0.005;
    umin_dt=-0.005;
    Umax=kron(ones(Nc,1),umax);
    Umin=kron(ones(Nc,1),umin);
    Umax_dt=kron(ones(Nc,1),umax_dt);
    Umin_dt=kron(ones(Nc,1),umin_dt);
    A_cons_cell=cell(2,2);
    A_cons_cell{1,1}=A_I;
    A_cons_cell{1,2}=zeros(Nu*Nc,1);
    A_cons_cell{2,1}=-A_I;
    A_cons_cell{2,2}=zeros(Nu*Nc,1);
    A_cons=cell2mat(A_cons_cell);
    B_cons_cell=cell(2,1);
    B_cons_cell{1,1}=Umax-Ut;
    B_cons_cell{2,1}=-Umin+Ut;
    B_cons=cell2mat(B_cons_cell);
    lb=Umin_dt;
    ub=Umax_dt;
    
    options=optimset('Algorithm','interior-point-convex');
    [X,~,~] =quadprog(H,f,A_cons,B_cons,[],[],lb,ub,[],options);
    
    u_piao=X(1);
    U=kesi(5)+u_piao;
    u_real=U;
    
    sys = u_real;