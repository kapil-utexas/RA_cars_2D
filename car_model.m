%state space 2-D model for a car
%This model has position co-ordinates x and y with velocity and theta, which define the state of the model
%The complete state space model and it's iterations will be computed
%idea is to iterate over the possible set of values and come up with the reachable states from the present state

DIVX = 10;
DIVY = 15;
DIVVEL = 30;
DIVTHETA = 20;
DIVACC = 10;
DIVRR = 60;
X0 = 2;
Y0 = 3;
VEL0 = 4;
THETA0 = 0;

XMIN = -200;
XMAX = 200;
YMIN = -100; 
YMAX = 100;
VELMIN = 0;
VELMAX = 30;
THETAMIN = -pi/4;
THETAMAX = pi/4;
ACCMIN = -10;
ACCMAX = -10;
RRMIN = -pi/8;
RRMAX = pi/8;

MAX_TIME = 10;
MIN_TIME = 0;
deltaT   = 1;
timeDiv  = (MAX_TIME-MIN_TIME)/deltaT ;

reachable_set = Inf(DIVX,DIVY,DIVVEL,DIVTHETA);

% initial state / conditions
xinit     = floor ((X0-XMIN)/(XMAX-XMIN)*(DIVX -1) +1);
yinit     = floor ((Y0-YMIN)/(YMAX-YMIN)*(DIVY -1) +1) ;
velinit   = floor ((VEL0-VELMIN)/(VELMAX-VELMIN)*(DIVVEL -1) +1)  ;
thetainit = floor ((THETA0-THETAMIN)/(THETAMAX-THETAMIN)*(DIVTHETA -1) +1) ;

reachable_set (xinit,yinit,velinit,thetainit) = 0; 
next_state = [inf,inf,inf,inf];
current_state = [xinit,yinit,velinit,thetainit];
prev_state = [0,0,0,0];
%prevTime(xinit,yinit,velinit,thetainit) = 0;
%presentTime(xinit,yinit,velinit,thetainit) = 0;
pos_X     = linspace(XMIN,XMAX,DIVX);               % in meters
pos_Y     = linspace(YMIN,YMAX,DIVY);               % in meters
pos_vel   = linspace(VELMIN,VELMAX,DIVVEL);         % in m/s
pos_theta = linspace(THETAMIN,THETAMAX,DIVTHETA);   % angle in radians
time      = linspace(MIN_TIME,MAX_TIME,timeDiv);    % time in seconds
pos_acc   = linspace(ACCMIN,ACCMAX,DIVACC);         % acceleration in m/s^2 , it is a control parameter
pos_rr    = linspace(RRMIN,RRMAX,DIVRR);            % rotation rate or omega in radians/sec, this is also a control parameter

%State Space Matrix updation for each time step
for i = 1:timeDiv                                                % for time
        for j = 1:DIVX                                           % for X
                for k = 1:DIVY                                   % for Y
                        for l = 1:DIVVEL                         % for vel    
                                for m = 1:DIVTHETA               %for theta
                                        for n = 1: DIVACC        %for accel
                                                for o = 1:DIVRR  %for Rotation Rate 
                                                       % prev_state = [j,k,l,m];
                                                        [a,b,c,d] =  evaluate_next_state(pos_X(j),pos_Y(k),pos_vel(l),pos_theta(m),pos_acc(n),pos_rr(o),deltaT);
                                                        next_state = [a,b,c,d];
                                                        x_ind      = floor ((a-XMIN)/(XMAX-XMIN)*(DIVX -1) +1);
                                                        y_ind      = floor ((b-YMIN)/(YMAX-YMIN)*(DIVY -1) +1);
                                                        vel_ind    = floor ((c-VELMIN)/(VELMAX-VELMIN)*(DIVVEL -1) +1);
                                                        theta_ind  = floor ((d-THETAMIN)/(THETAMAX-THETAMIN)*(DIVTHETA -1) +1);
                                                        if x_ind > 0 && x_ind <= DIVX && y_ind > 0 && y_ind <= DIVY && vel_ind > 0 && vel_ind <= DIVVEL && theta_ind > 0 && theta_ind <= DIVTHETA
                                                                reachable_set(x_ind,y_ind,vel_ind,theta_ind) = min( (deltaT + reachable_set(j,k,l,m) ) , reachable_set(x_ind,y_ind,vel_ind,theta_ind) );
                                                        end
                                                end
                                        end
                                end
                        end
                end
        end
end
%contour3(reachable_set(:,:,5,5),time);




