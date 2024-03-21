classdef MPCController < matlab.System
    % This class defines a custom System object for an MPC controller used for path tracking.
    
    properties
        % Define properties for the MPC controller
        Nx = 5;
        Nu = 1;
        Ny = 2;
        Np = 20;
        Nc = 10;
        T = 0.01;
        lf = 1.488;
        lr = 1.487;
        Ccf = 66900;
        Ccr = 62700;
        m = 1600;
        g = 9.81;
        I = 2986.624;
        x_dot = 15;
        Q = diag([2000 0 0 10000 0]);
        R = 5 * 10^5;
        Row = 1000;
        umin = -0.6;
        umax = 0.6;
        delta_umin = -0.06;
        delta_umax = 0.06;
        ycmax = [0.2; 1000];
        ycmin = [-0.2; -1000];
    end
    
    properties (DiscreteState)
        % Define discrete state for the MPC controller
        x % State vector
        delta_f % Steering angle
    end

    methods (Access = protected)
        function setupImpl(obj)
        function num = getNumInputsImpl(~)
            % Define number of inputs for the System object
            num = 5;
        end
        
        function num = getNumOutputsImpl(~)
            % Define number of outputs for the System object
            num = 2;
        end
        end
        
        function [delta_f, state_k1] = stepImpl(obj, y_dot, phi, phi_dot, Y, X)
            % Define the step method for the System object
            % This method corresponds to the mdlOutputs function
            
            % Define system matrices and parameters
            a = [1 - (1296 * obj.T) / 127, 0, -(203087 * obj.T) / 12700, 0, 0;
                 0, 1, obj.T, 0, 0;
                 (50913 * obj.T) / 15367, 0, 1 - (7351983 * obj.T) / 384175, 0, 0;
                 obj.T * cos(phi), obj.T * (20 * cos(phi) - y_dot * sin(phi)), 0, 1, 0;
                 -obj.T * sin(phi), -obj.T * (20 * sin(phi) + y_dot * cos(phi)), 0, 0, 1];
            b = [(13380 * obj.T) / 127;
                 0;
                 (1358070 * obj.T) / 15367;
                 0;
                 0];
            C = [0 1 0 0 0;
                 0 0 0 1 0];
            d_k = zeros(obj.Nx, 1);
            
            % Calculate the next state
            state_k1 = zeros(obj.Nx, 1);
            state_k1(1,1) = y_dot + obj.T * (-obj.x_dot * phi_dot + 2 * (obj.Ccf * (obj.delta_f - (y_dot + obj.lf * phi_dot) / obj.x_dot) + obj.Ccr * (obj.lr * phi_dot - y_dot) / obj.x_dot) / obj.m);
            state_k1(2,1) = phi + obj.T * phi_dot;
            state_k1(3,1) = phi_dot + obj.T * ((2 * obj.lf * obj.Ccf * (obj.delta_f - (y_dot + obj.lf * phi_dot) / obj.x_dot) - 2 * obj.lr * obj.Ccr * (obj.lr * phi_dot - y_dot) / obj.x_dot) / obj.I);
            state_k1(4,1) = Y + obj.T * (obj.x_dot * sin(phi) + y_dot * cos(phi));
            state_k1(5,1) = X + obj.T * (obj.x_dot * cos(phi) - y_dot * sin(phi));
            d_k = state_k1 - a * obj.x - b * obj.delta_f;
            
            % Define matrices for optimization
            PSI = zeros(obj.Ny * obj.Np, obj.Nx);
            THETA = zeros(obj.Ny * obj.Np, obj.Nu * obj.Nc);
            GAMMA = zeros(obj.Ny * obj.Np, obj.Nx);
            PHI = zeros(obj.Ny * obj.Np, 1);
            H = zeros(obj.Nu * obj.Nc + 1, obj.Nu * obj.Nc + 1);
            f = zeros(obj.Nu * obj.Nc + 1, 1);
            A_I = zeros(obj.Nc, obj.Nc);
            Umin = zeros(obj.Nc, 1);
            Umax = zeros(obj.Nc, 1);
            Ycmin = zeros(obj.Np, 1);
            Ycmax = zeros(obj.Np, 1);
            Yita_ref = zeros(obj.Np, 1);
            X_predict = zeros(obj.Np, 1);
            Y_ref = zeros(obj.Np, 1);
            phi_ref = zeros(obj.Np, 1);
            T_all = 2.5;
            xf = 5000;
            U(1) = kesi(6,1);
            % Calculate reference trajectory
            for p = 1:1:obj.Np
                if obj.T + p * obj.T > T_all
                    X_predict(obj.Np,1) = X + obj.x_dot * obj.Np * obj.T;
                    Y_ref(p,1) = Y;
                    phi_ref(p,1) = 0;
                    Yita_ref(p,1) = Y_ref(p,1);
                else
                    X_predict(p,1) = X + obj.x_dot * p * obj.T;
                    Y_ref(p,1) = Y*10 * (X_predict(p,1) / xf)^3 - 15 * Y * (X_predict(p,1) / xf)^4 + 6 * Y * (X_predict(p,1) / xf)^5;
                    phi_ref(p,1) = atan((9 * X_predict(obj.Np,1)^4) / 25000000 - (9 * X_predict(obj.Np,1)^3) / 250000 + (9 * X_predict(obj.Np,1)^2) / 10000);
                    Yita_ref(p,1) = Y_ref(p,1);
                end
            end
            
            % Construct the matrices for optimization
            for j = 1:1:obj.Np
                PSI((j-1)*obj.Ny+1:j*obj.Ny,:) = C * a^j; % Calculate PSI for each time step
                GAMMA((j-1)*obj.Ny+1:j*obj.Ny,:) = C * a^(j-1); % Calculate GAMMA for each time step
                PHI((j-1)*obj.Ny+1:j*obj.Ny,:) = C * d_k; % Calculate PHI for each time step
                Ycmin((j-1)*obj.Ny+1:j*obj.Ny,:) = obj.ycmin - Yita_ref(j,:); % Calculate Ycmin for each time step
                Ycmax((j-1)*obj.Ny+1:j*obj.Ny,:) = obj.ycmax - Yita_ref(j,:); % Calculate Ycmax for each time step
                
                % Calculate THETA for each time step and control horizon
                for k = 1:1:obj.Nc
                    if k <= j
                        THETA((j-1)*obj.Ny+1:j*obj.Ny,(k-1)*obj.Nu+1:k*obj.Nu) = C * a^(j-k) * b; % If k <= j, calculate C * a^(j-k) * b
                    else
                        THETA((j-1)*obj.Ny+1:j*obj.Ny,(k-1)*obj.Nu+1:k*obj.Nu) = zeros(obj.Ny,obj.Nu); % If k > j, assign zeros
                    end
                end
            end
            
            % Construct the Hessian matrix H
            H(1:obj.Nu*obj.Nc,1:obj.Nu*obj.Nc) = 2 * THETA' * obj.Q * THETA + obj.R * eye(obj.Nu*obj.Nc);
            H(obj.Nu*obj.Nc+1,obj.Nu*obj.Nc+1) = obj.Row;
            
            % Construct the vector f
            f(1:obj.Nu*obj.Nc) = -2 * (Yita_ref - PSI * obj.x - GAMMA * d_k)' * obj.Q * THETA;
            
            % Construct the constraint matrices
            for p = 1:1:obj.Nc
                for q = 1:1:obj.Nc
                    if q <= p
                        A_I(p,q) = 1;
                    else
                        A_I(p,q) = 0;
                    end
                end
            end
            Umin = kron(ones(obj.Nc,1), obj.umin) - A_I * obj.delta_f;
         Umax = kron(ones(obj.Nc,1), obj.umax) - A_Ikesi(1) * obj.delta_f;
            Ut = kron(ones(obj.Nc,1), U(1));
            % Construct inequality constraints
            A_cons_cell = {A_I zeros(obj.Nu * obj.Nc, 1); -A_I zeros(obj.Nu * obj.Nc, 1); THETA zeros(obj.Ny * obj.Np, 1); -THETA zeros(obj.Ny * obj.Np, 1)};
            b_cons_cell = {Umax - Ut; -Umin + Ut; Ycmax - PSI * kesi - GAMMA * PHI; -Ycmin + PSI * kesi + GAMMA * PHI};
            A_cons = cell2mat(A_cons_cell);
            b_cons = cell2mat(b_cons_cell);
            
            % Set up optimization problem
            M = 10;
            delta_Umin = kron(ones(obj.Nc,1), obj.delta_umin);
            delta_Umax = kron(ones(obj.Nc,1), obj.delta_umax);
            lb = [delta_Umin; 0];
            ub = [delta_Umax; M];
            options = optimoptions('quadprog','Display','off');
            
            % Solve quadratic programming problem
            [X, fval, exitflag] = quadprog(H, f, A_cons, b_cons, [], [], lb, ub, [], options);
            u_piao(1) = X(1);
            U(1) = kesi(6,1) + u_piao(1);
            delta_f = U(1);
            sys = U;

        end
        
        function resetImpl(obj)
            % Define the reset method for the System object
            % This method initializes the state vector and the steering angle
            obj.x = [0.0001;0.0001;0.00001;0.00001;0.00001];
            obj.delta_f = 0;
        end
       
    end
end
