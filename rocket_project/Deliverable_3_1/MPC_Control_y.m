classdef MPC_Control_y < MPC_Control
    
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
            Q =eye(nx);
            R = eye(nu);
            
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            K = -K;
            
            %delta2 constrains
            Hu = [1;-1];
            ku = [deg2rad(15);deg2rad(15)];
            
            %state constrains
            Hx = [0 1 0 0; 0 -1 0 0];
            kx = [deg2rad(5);deg2rad(5)];
            
            %compute the terminal set we will use that ensure the condtions
            %compute max invariant set in closed loop with LQR terminal controller
            Xf = polytope([Hx; Hu*K],[kx;ku]);
            %X+ = Ax + Bu = Ax + BKx = (A+BK)x
            Acl = mpc.A+mpc.B*K; 
            while 1
                Xf_ = Xf;
                [terminal_setM,n] = double(Xf);
                preXf = polytope(terminal_setM*Acl,n);
                Xf = intersect(Xf,preXf);
                if Xf == Xf_
                    break
                end
            end
            %get the terminal set !
            [terminal_setM,terminal_setv] = double(Xf);
            figure
            hold on
            plot(Xf.projection(1:2),'g');
            xlabel('wx'); ylabel('alpha');
            figure
            hold on
            plot(Xf.projection(2:3),'r');
            xlabel('alpha'); ylabel('Vy');
            figure
            hold on
            plot(Xf.projection(3:4),'b');
            xlabel('Vy'); ylabel('Y');


            %set up full controller
            con = (X(:,2) == mpc.A*X(:,1)+mpc.B*U(:,1)) + (Hu*U(:,1)<=ku);
            con = con + (Hx*X(:,1)<=kx); %not necessary I think
            obj = U(:,1)'*R*U(:,1); %not putting x0 in it, as it is a constant (does not change anything to add it actually I think)
            for i = 2:N-1
                con = con + (X(:,i+1)==mpc.A*X(:,i)+mpc.B*U(:,i)) + (Hx*X(:,i) <= kx) + (Hu*U(:,i)<=ku); %constrain on xi, ui !
                obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);
            end
            obj = obj + X(:,N)'*Qf*X(:,N); %final stage cost
            con = con + (terminal_setM*X(:,N) <= terminal_setv); %Xn in Xf (terminal set)
        
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
            obj = 0;
            con = [xs == 0, us == 0];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
