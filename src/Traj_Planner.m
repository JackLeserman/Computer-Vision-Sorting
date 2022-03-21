classdef Traj_Planner < handle
    methods
        % Takes in starting and ending times, starting and ending velocities,
        % starting and ending positions, and outpits a 4x1 array containing the
        % coefficients ai i = 0,1,2,3 of the polynomials
        function trajectory = cubic_traj(self,t0,tf,v0,vf,q0,qf)
            M = [1 t0 t0.^2 t0.^3; 0 1 2*t0 3*t0.^2; 1 tf tf.^2 tf.^3; 0 1 2*tf 3*tf.^2];
            %M = [1 t0 power(t0,2) power(t0,3); 0 1 2*t0 power(3*t0,2); 1 tf power(tf,2) power(tf,2); 0 1 2*tf power(3*tf,3)]
            Q = [q0; v0; qf; vf];
            trajectoryR = inv(M)*Q;
            trajectory = transpose(trajectoryR)
        end

        % 
        function trajectory = quintic_traj(self,t0, tf, v0, vf, q0, qf, alpha0, alphaf)
            M = [1 t0 t0^2 t0^3 t0^4 t0^5; 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 0 0 2 6*t0 ...
                12*t0^2 2*t0^3; 1 tf tf^2 tf^3 tf^4 tf^5; 0 1 2*tf 3*tf^2 4*tf^3 ... 
                5*tf^4; 0 0 2 6*tf 12*tf^2 20*tf^3];
            Q = [q0; v0; alpha0; qf; vf; alphaf];
            trajectoryR = inv(M)*Q;
            trajectory = transpose(trajectoryR);
        end
    end
end