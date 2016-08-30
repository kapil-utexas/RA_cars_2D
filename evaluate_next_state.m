%This function will be used to evaluate 
%This model has position co-ordinates x and y with velocity and theta, which define the state of the model
%input is current state parameters, time granularity & control inputs and the output is the next state

function [x,y,v,theta] = evaluate_next_state(xi,yi,vi,thetai,acc,rr,deltaT)
%Newton's laws of motion in 2D space
       x     = xi      + vi*cos(thetai)*deltaT;
       y     = yi      + vi*sin(thetai)*deltaT;
       v     = vi      + acc*deltaT;
       theta = thetai  + rr*deltaT;
end

