classdef MPC_Control_z < MPC_Control
    properties
        A_bar, B_bar, C_bar % Augmented system for disturbance rejection
        L                   % Estimator gain for disturbance rejection
    end
    
    methods
        function mpc = MPC_Control_z(sys, Ts, H)
            mpc = mpc@MPC_Control(sys, Ts, H);
            
            [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
        end
        
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   d_est        - disturbance estimate
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps
            
            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.3)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar(1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

            Q = [1 0;
                 0 500];
            R =0.01*eye(nu);
            %R = zeros(nu);

                        
            [~,Qf,~] = dlqr(mpc.A,mpc.B,Q,R); 
            %state constrains: none
            
            %Pavg constrains
            Hu = [1;-1];
            ku = [80 - 56.6667;-50 + 56.6667];

            %set up full controller
            con = ((X(:,2)-x_ref) == mpc.A*(X(:,1)-x_ref)+mpc.B*(U(:,1)-u_ref))...
                + (Hu*(U(:,1)-u_ref)<=(ku - Hu*u_ref));
              
           
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref); %not putting x0 in it, as it is a constant (does not change anything to add it actually I think)
            for i = 2:N-1
                con = con + ((X(:,i+1)-x_ref)==mpc.A*(X(:,i)-x_ref)+mpc.B*(U(:,i)-u_ref))...
                    + (Hu*(U(:,i)-u_ref)<=(ku - Hu*u_ref)); %constrain on ui !
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref); %final stage cost
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref, d_est}, U(:,1));
        end
        
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);
            
            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.3)
            ref = sdpvar;
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            R = eye(1);
            %Pavg constrains
            Hu = [1;-1];
            ku = [80 - 56.6667;-50 + 56.6667];
            
            %state constrains: none
            
            obj = us'*R*us;
            M = [eye(size(mpc.A))-mpc.A -mpc.B;
                 mpc.C 0];
            
            con = (M*[xs;us]==[mpc.B*d_est;ref])...
                 + (Hu*us<=ku);
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
        end
        
        
        % Compute augmented system and estimator gain for input disturbance rejection
        function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
            
            %%% Design the matrices A_bar, B_bar, L, and C_bar
            %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
            %%% converges to the correct state and constant input disturbance
            %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            nx = size(mpc.A,1);
            nu = size(mpc.B,2);
            nd = 1;
            
            A_bar = [mpc.A mpc.B;
                     zeros(nd,nx) eye(nd,nu)];
            B_bar = [mpc.B; zeros(1,nu)];
            C_bar = [mpc.C zeros(1,1)];
            
            %Use optimal LQR observer for smooth behaviour
            %Or place them to 0 ... hmmm ==> 
            L = -place(A_bar',C_bar',[0.8,0.5,0.7])';
% 
            %Q1 = eye(nx+nd);
%             Q1 = [1 0 0; 0 1 0; 0 0 1000];
%             Q2 = eye(size(C_bar,1));
%             [L,~,~]=dlqr(A_bar',C_bar',Q1,Q2);
%             L = -L';
%             eig(A_bar+L*C_bar)
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        
    end
end
