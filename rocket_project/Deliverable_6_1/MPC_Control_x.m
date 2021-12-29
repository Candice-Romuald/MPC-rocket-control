classdef MPC_Control_x < MPC_Control
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N = ceil(H/Ts); % Horizon steps

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            Q =diag([1000 100000 10 100]); %X weight = state weights = wy, B, vx, x
            %R = eye(nu); %Energy weight = input weight
            R = zeros(nu);

            
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            K = -K;
            
            %delta2 constrains
            Hu = [1;-1];
            ku = [deg2rad(15);deg2rad(15)];
            
            %state constrains
            Hx = [0 1 0 0; 0 -1 0 0];
            kx = [deg2rad(5);deg2rad(5)];
            
            obj = 0;

            %set up full controller
            con = (X(:,2)- x_ref == mpc.A*(X(:,1)-x_ref)+mpc.B*(U(:,1)-u_ref))...
            + (Hu*(U(:,1)-u_ref)<=ku-Hu*u_ref);
            for i = 2:N-1
                con = con + (X(:,i+1)-x_ref==mpc.A*(X(:,i)-x_ref)+mpc.B*(U(:,i)-u_ref))...
                    + (Hx*(X(:,i)-x_ref) <= kx - Hx*x_ref) + (Hu*(U(:,i)-u_ref)<=ku-Hu*u_ref); %constrain on xi, ui !
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref);
            end
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref); %final stage cost
            con = con + (Hx*(X(:,N)-x_ref) <= kx - Hx*x_ref); %to add
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, U(:,1));
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            R = eye(1);
            %delta2 constrains
            Hu = [1;-1];
            ku = [deg2rad(15);deg2rad(15)];
            
            %state constrains
            Hx = [0 1 0 0; 0 -1 0 0];
            kx = [deg2rad(5);deg2rad(5)];
            
            obj = us'*R*us;
            M = [eye(size(mpc.A))-mpc.A -mpc.B;
                 mpc.C 0];
            
            con = (M*[xs;us]==[zeros(nx,1);ref])...
                 + (Hx*xs<=kx)...
                 + (Hu*us<=ku);
            %con = [xs == 0, us == 0];
            
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
