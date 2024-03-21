function [sys, x0, str, ts] = MPCController1(t, x, u, flag)
% This function defines an S-function for an MPC controller used for path tracking.
% It switches behavior based on the value of the 'flag' argument.

switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes; % Initialize the sizes
    case 2
        sys = mdlUpdates(t, x, u); % Update the model
    case 3
        sys = mdlOutputs(t, x, u); % Output the model
    case {1, 4, 9}
        sys = []; % Do nothing for other cases
    otherwise
        error(['Unhandled flag=', num2str(flag)]); % Handle errors
end

    function sys = mdlUpdates(~, x, u)
        % This function updates the model.
        sys = x; % Update the model state
    end
    function [sys, x0, str, ts] = mdlInitializeSizes
        % This function initializes the sizes of the S-function.
        sizes = simsizes;
        sizes.NumContStates = 0;
        sizes.NumDiscStates = 5;
        sizes.NumOutputs = 1;
        sizes.NumInputs = 8;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0 = [0.0001; 0.0001; 0.00001; 0.00001; 0.00001]; % Initial state values
        global U;
        U=[0]; % Initialize global variable U
        str = []; % Set str to an empty matrix
        ts = [0.01, 0]; % Sample time: [period, offset]

    end
    function sys = mdlOutputs(t, ~, u)
        % This function outputs the model.

        % Define model parameters and variables
        %global U;
        global x_dot;
        %global xf;
        Nx = 5;
        Nu = 1;
        Ny = 2;
        Np = 20;
        Nc = 10;
        T = 0.01;
        y_dot = u(1) / 3.6;
        phi = u(2) * 3.141592654 / 180;
        phi_dot = u(3) * 3.141592654 / 180;
        Y = u(4);
        X = u(5);
        x_dot = u(8);
        U = [0];
        lf = 1.488;
        lr = 1.487;
        Ccf = 6.1525e+04;
        Ccr = 31054.23;
        m = 1600;
        g = 9.81;
        I = 2986.624;

        % Define other parameters...
        X_predict = zeros(Np, 1);
        phi_ref = zeros(Np, 1);
        Y_ref = zeros(Np, 1);
        kesi = zeros(Nx + Nu, 1);
        kesi(1) = y_dot;
        kesi(2) = phi;
        kesi(3) = phi_dot;
        kesi(4) = Y;
        kesi(5) = X;
        kesi(6) = U(1);
        delta_f = U(1);
        u_piao = zeros(Nx, Nu);
        Q_cell = cell(Np, Np);
        for i = 1:1:Np
            for j = 1:1:Np
                if i == j
                    Q_cell{i,j} = [2000 0; 0 10000]; % Diagonal elements of Q matrix
                else
                    Q_cell{i,j} = zeros(Ny, Ny); % Off-diagonal elements of Q matrix
                end
            end
        end

        R = 5 * 10^5 * eye(Nu * Nc); % Define R matrix

        % Define system matrices and parameters
        Row = 1000;
        a = [1 - ((Ccf+Ccr) * T) / (1600), 0, -((lf*Ccf-lr*Ccr+m*x_dot^2) * T) / (1600*x_dot/2), 0, 0;
            0, 1, T, 0, 0;
            ((lf*Ccf-lr*Ccr) * T) / (I*x_dot/2), 0, 1-(2.5*Ccf*lf^2 + 2.5*Ccr*lr^2)/(I*x_dot/2*2.5*10), 0, 0;
            T * cos(phi),  T * (x_dot * cos(phi) - y_dot * sin(phi)), 0, 1, 0;
            -T * sin(phi), -T * (x_dot * sin(phi) + y_dot * cos(phi)), 0, 0, 1];
        b = [(2*Ccf * T) / m;
            0;
            ( 2*Ccf*lf *10* T) / (I*x_dot/2);
            0;
            0];

        % Initialize state and define system matrices
        A_cell = cell(2, 2);
        B_cell = cell(2, 1);
        A_cell{1,1} = a;
        A_cell{1,2} = b;
        A_cell{2,1} = zeros(Nu, Nx);
        A_cell{2,2} = eye(Nu);
        B_cell{1,1} = b;
        B_cell{2,1} = eye(Nu);
        A = cell2mat(A_cell);
        B = cell2mat(B_cell);
        C = [0 1 0 0 0 0;
            0 0 0 1 0 0];
        d_k = zeros(Nx, 1);

        % Calculate the next state
        state_k1 = zeros(Nx, 1);
        state_k1(1,1) = y_dot + T * (-x_dot * phi_dot + 2 * (Ccf * (delta_f - (y_dot + lf * phi_dot) / x_dot) + Ccr * (lr * phi_dot - y_dot) / x_dot) / m);
        state_k1(2,1) = phi + T * phi_dot;
        state_k1(3,1) = phi_dot + T * ((2 * lf * Ccf * (delta_f - (y_dot + lf * phi_dot) / x_dot) - 2 * lr * Ccr * (lr * phi_dot - y_dot) / x_dot) / I);
        state_k1(4,1) = Y + T * (x_dot * sin(phi) + y_dot * cos(phi));
        state_k1(5,1) = X + T * (x_dot * cos(phi) - y_dot * sin(phi));
        d_k = state_k1 - a * kesi(1:5,1) - b * kesi(6,1);
        d_piao_k = zeros(Nx + Nu, 1);
        d_piao_k(1:5,1) = d_k;
        d_piao_k(6,1) = 0;

        % Define matrices for optimization
        PSI_cell = cell(Np, 1);
        THETA_cell = cell(Np, Nc);
        GAMMA_cell = cell(Np, Np);
        PHI_cell = cell(Np, 1);
        % Initialize PHI_cell with d_piao_k for each time step
        for p = 1:1:Np
            PHI_cell{p,1} = d_piao_k;

            % Initialize GAMMA_cell based on the time step
            for q = 1:1:Np
                if q <= p
                    GAMMA_cell{p,q} = C * A^(p-q); % If q <= p, calculate C * A^(p-q)
                else
                    GAMMA_cell{p,q} = zeros(Ny,Nx+Nu); % If q > p, assign zeros
                end
            end
        end

        % Initialize PSI_cell and THETA_cell based on the time step and prediction horizon
        for j = 1:1:Np
            PSI_cell{j,1} = C * A^j; % Calculate PSI for each time step

            % Initialize THETA_cell based on the time step and control horizon
            for k = 1:1:Nc
                if k <= j
                    THETA_cell{j,k} = C * A^(j-k) * B; % If k <= j, calculate C * A^(j-k) * B
                else
                    THETA_cell{j,k} = zeros(Ny,Nu); % If k > j, assign zeros
                end
            end
        end

        % Convert cell arrays to matrices
        PSI = cell2mat(PSI_cell); % size(PSI) = [Ny*Np Nu*Nc]
        THETA = cell2mat(THETA_cell);
        GAMMA = cell2mat(GAMMA_cell);
        PHI = cell2mat(PHI_cell);
        Q = cell2mat(Q_cell);

        % Construct the Hessian matrix H
        H_cell = cell(2,2);
        H_cell{1,1} = 2 * THETA' * Q * THETA + R;
        H_cell{1,2} = zeros(Nu*Nc,1);
        H_cell{2,1} = zeros(1,Nu*Nc);
        H_cell{2,2} = Row;
        H = cell2mat(H_cell);

        % Initialize error_1 matrix
        error_1 = zeros(Ny*Np,1);
        T_all=2.5;
        global xf;
        xf=50;
        % Calculate reference trajectory
        for p = 1:1:Np
            if t + p * T > T_all
                X_predict(Np,1) = X + x_dot * Np * T;
                Y_ref(p,1) = u(6);
                phi_ref(p,1) = (u(7))*pi()/180;%u(2);
                Yita_ref_cell{p,1} = [phi_ref(p,1); Y_ref(p,1)];
            else
                X_predict(p,1) = X + x_dot * p * T;
                Y_ref(p,1) = u(6); %37.5 * (X_predict(p,1) / xf)^3 - 15 * 3.75 * (X_predict(p,1) / xf)^4 + 6 * 3.75* (X_predict(p,1) / xf)^5;
                phi_ref(p,1) = u(7)*pi()/180;%atan((9 * X_predict(Np,1)^4) / 25000000 - (9 * X_predict(Np,1)^3) / 250000 + (9 * X_predict(Np,1)^2) / 10000);
                Yita_ref_cell{p,1} = [phi_ref(p,1); Y_ref(p,1)];
            end
        end
        % Calculate reference trajectory
        Yita_ref = cell2mat(Yita_ref_cell);
        error_1 = (Yita_ref - PSI * kesi - GAMMA * PHI);
        f_cell = cell(1,2);
        f_cell{1,1} = 2 * error_1' * Q * THETA;
        f_cell{1,2} = 0;
        f = -cell2mat(f_cell);

        % Define constraints
        A_t = zeros(Nc, Nc);
        for p = 1:1:Nc
            for q = 1:1:Nc
                if q <= p
                    A_t(p,q) = 1;
                else
                    A_t(p,q) = 0;
                end
            end
        end
        A_I = kron(A_t, eye(Nu));
        Ut = kron(ones(Nc,1), U(1));
        umin = -1.8;
        umax = 1.8;
        delta_umin = -0.18;
        delta_umax = 0.18;
        Umin = kron(ones(Nc,1), umin);
        Umax = kron(ones(Nc,1), umax);
        ycmax = [0.002; 1000];
        ycmin = [-0.002; -1000];
        Ycmax = kron(ones(Np,1), ycmax);
        Ycmin = kron(ones(Np,1), ycmin);

        % Construct inequality constraints
        A_cons_cell = {A_I zeros(Nu * Nc, 1); -A_I zeros(Nu * Nc, 1); THETA zeros(Ny * Np, 1); -THETA zeros(Ny * Np, 1)};
        b_cons_cell = {Umax - Ut; -Umin + Ut; Ycmax - PSI * kesi - GAMMA * PHI; -Ycmin + PSI * kesi + GAMMA * PHI};
        A_cons = cell2mat(A_cons_cell);
        b_cons = cell2mat(b_cons_cell);

        % Set up optimization problem
        M = 10;
        delta_Umin = kron(ones(Nc,1), delta_umin);
        delta_Umax = kron(ones(Nc,1), delta_umax);
        % Define matrices THETA, PSI, GAMMA, PHI, Q...

        % Solve constrained optimization problem using fmincon
        lb = [delta_Umin; 0];
        ub = [delta_Umax; M];
        options = optimset('Display','off','Algorithm', 'interior-point');  % Adjust options as needed
        initialGuess = zeros(Nu * Nc + 1, 1);  % Initial guess for optimization variables

        [X, ~, exitflag] = fmincon(@(x) objectiveFunction(x, Q, THETA, PSI, kesi, GAMMA, PHI), ...
            initialGuess, A_cons, b_cons, [], [], lb, ub, [], options);
        
        u_piao = X(1:Nu);
        U(1) = kesi(6, 1) + u_piao(1);
        sys = U;
        function cost = objectiveFunction(x, Q, THETA, PSI, kesi, GAMMA, PHI)
            % Define objective function to minimize (quadratic cost function)
            u_piao = x(1:Nu);  % Extract control input increments from optimization variables
            error_2 = (PSI * kesi + GAMMA * PHI + THETA * u_piao);  % Compute prediction error
            cost = norm(error_2' * Q * error_2);  % Compute quadratic cost
            cost
        end
    end
end

