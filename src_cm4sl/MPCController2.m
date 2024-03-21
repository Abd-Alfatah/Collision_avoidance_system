function [sys, x0, str, ts] = MPCController2(t, x, u, flag)
    % This function defines an S-function for an MPC controller used for path tracking.
    % It switches behavior based on the value of the 'flag' argument.

    % Initialize sizes and sample time
    switch flag
        case 0
            [sys, x0, str, ts] = mdlInitializeSizes();
        case 2
            sys = mdlUpdates(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        otherwise
            sys = [];
    end

    function [sys, x0, str, ts] = mdlInitializeSizes()
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
        str = [];
        ts = [0.01, 0]; % Sample time: [period, offset]
    end

    function sys = mdlUpdates(~, x, ~)
        % This function updates the model.
        sys = x; % Update the model state
    end

    function sys = mdlOutputs(~, ~, u)
        % This function outputs the model.
        global U;
        tic; % Start timer

        % Define model parameters and variables
        Nx = 5;
        Nu = 1;
        Ny = 2;
        Np = 20;
        Nc = 10;
        T = 0.01;
        y_dot = u(1) / 3.6;
        phi = u(2) * pi / 180;
        phi_dot = u(3) * pi / 180;
        Y = u(4);
        X = u(5);
        global x_dot;
        x_dot = u(8);
        lf = 1.488;
        lr = 1.487;
        Ccf = 6.1525e+04;
        Ccr = 31054.23;
        m = 1600;
        g = 9.81;
        I = 2986.624;
        X_predict = zeros(Np, 1);
        phi_ref = zeros(Np, 1);
        Y_ref = zeros(Np, 1);
        kesi = [y_dot; phi; phi_dot; Y; X; U(1)];
        delta_f = U(1);
        u_piao = zeros(Nx, Nu);
        Q_cell = cell(Np, Np);
        for i = 1:Np
            for j = 1:Np
                if i == j
                    Q_cell{i,j} = [2000, 0; 0, 10000]; % Diagonal elements of Q matrix
                else
                    Q_cell{i,j} = zeros(Ny, Ny); % Off-diagonal elements of Q matrix
                end
            end
        end

        R = 5e5 * eye(Nu * Nc); % Define R matrix

        % Define system matrices and parameters
        a = [1 - ((Ccf + Ccr) * T) / (1600), 0, -((lf * Ccf - lr * Ccr + m * x_dot^2) * T) / (1600 * x_dot / 2), 0, 0;
            0, 1, T, 0, 0;
            ((lf * Ccf - lr * Ccr) * T) / (I * x_dot / 2), 0, 1 - (2.5 * Ccf * lf^2 + 2.5 * Ccr * lr^2) / (I * x_dot / 2 * 2.5 * 10), 0, 0;
            T * cos(phi), T * (x_dot * cos(phi) - y_dot * sin(phi)), 0, 1, 0;
            -T * sin(phi), -T * (x_dot * sin(phi) + y_dot * cos(phi)), 0, 0, 1];
        b = [(2 * Ccf * T) / m;
            0;
            (2 * Ccf * lf * 10 * T) / (I * x_dot / 2);
            0;
            0];
            C = [0 1 0 0  0;
            0 0 0 1 0 ];
        % Calculate the next state
        state_k1 = [y_dot + T * (-x_dot * phi_dot + 2 * (Ccf * (delta_f - (y_dot + lf * phi_dot) / x_dot) + Ccr * (lr * phi_dot - y_dot) / x_dot) / m);
                    phi + T * phi_dot;
                    phi_dot + T * ((2 * lf * Ccf * (delta_f - (y_dot + lf * phi_dot) / x_dot) - 2 * lr * Ccr * (lr * phi_dot - y_dot) / x_dot) / I);
                    Y + T * (x_dot * sin(phi) + y_dot * cos(phi));
                    X + T * (x_dot * cos(phi) - y_dot * sin(phi))];
        d_k = state_k1 - a * kesi(1:5) - b * kesi(6);
        d_piao_k = [d_k; 0];

        % Initialize matrices for optimization
        Q = blkdiag(Q_cell{:});
        PSI = [];
        THETA = [];
        GAMMA = [];
        PHI = d_piao_k;
        for j = 1:Np
            PSI = blkdiag(PSI, C * a^j);
            for k = 1:Nc
                if k <= j
                    THETA = [THETA; C * a^(j - k) * b];
                else
                    THETA = [THETA; zeros(Ny, Nu)];
                end
            end
            GAMMA = blkdiag(GAMMA, C * a^j);
        end
        Q = Q(1:Ny*Np*10, 1:Ny*Np*10);
        size(PSI)
        size(Q)
        size(kesi)
        size(GAMMA)
        size(PHI)
        % Construct Hessian matrix H
       H = [2 * THETA' * Q * THETA + R * eye(Nu * Nc), zeros(Nu * Nc, Nu); zeros(Nu, Nu * Nc), 1000];
        % Define inequality constraints
        %A_cons = [eye(Nu * Nc), zeros(Nu * Nc, 1); -eye(Nu * Nc), zeros(Nu * Nc, 1); THETA, zeros(Ny * Np, 1); -THETA, zeros(Ny * Np, 1)];
        %b_cons = [repmat(1.8, Nu * Nc, 1) - U(1); repmat(1.8, Nu * Nc, 1) + U(1); repmat(1000, Ny * Np, 1) - PSI * kesi - GAMMA * PHI; repmat(1000, Ny * Np, 1) + PSI * kesi + GAMMA * PHI];
        %A_cons
        % Set up optimization problem
        lb = [-repmat(0.18, Nu * Nc, 1); 0];
        ub = [repmat(0.18, Nu * Nc, 1); 10];
        options = optimset('Algorithm', 'interior-point-convex');
        %f = -2 * Q * (THETA' * (PSI * kesi + GAMMA * PHI));
        % Solve quadratic programming problem
        [X, ~, exitflag] = quadprog(H, [], [], [], [], [], lb, ub, [], options);
        u_piao(1) = X(1);
        U(1) = kesi(6) + u_piao(1);
        sys = U ; % Dividing by 100 as in your original code
    end
end
