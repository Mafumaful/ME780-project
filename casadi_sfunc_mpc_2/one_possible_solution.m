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
    U=[0];
    
    function sys=mdlUpdate(t,x,u)
    
    sys = x;
    
    function sys=mdlOutputs(t,x,u)
    %% 定义全局变量
    global A1 B1 u_piao U kesi
    Nx=4;%状态量4个，横向误差，航向角误差，侧向速度误差，横摆角速度误差
    Nu=1;%控制量1个前轮转角
    Np=100;%预测步长50
    Nc=80;%控制步长30
    Row=10;%松弛因子
    T=0.01;%采用时间0.01
    %%车辆参数
    m=1274+71*2;%整车质量
    a=1.015;%质心到前轴的距离
    b=1.895;%质心到后轴的距离
    Iz=1536.7;%绕Z轴的转动惯量
    k1=-112600;%前轴侧偏刚度
    k2=-89500;%后轴侧偏刚度
    %%控制器设计
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
       0,0,0,1];
    Q=kron(eye(Np),q);
    R=eye(Nc*Nu);
    A2=[0,vx,1,0;
       0,0,0,1;
       0,0,(k1+k2)/m/vx,(a*k1-b*k2)/m/vx-vx;
       0,0,(a*k1-b*k2)/Iz/vx,(a^2*k1+b^2*k2)/Iz/vx];
    B2=[0;0;-k1/m;-a*k1/Iz];
    A1=A2*T+eye(Nx);%雅可比矩阵计算
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
    %% 预测时域矩阵升维
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
    A_t=zeros(Nc,Nc);%工具人矩阵
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
    lb=[Umin_dt];
    ub=[Umax_dt];
    %% 二次规划问题
    options=optimset('Algorithm','interior-point-convex');
    [X,fval,exitflag] =quadprog(H,f,A_cons,B_cons,[],[],lb,ub,[],options);
    %% 赋值输出
    u_piao=X(1);
    U=kesi(5)+u_piao;
    u_real=U;
    
    sys = u_real;