inversekinematics
===============================
##What is Inverse Kinematic (IK)?

The inverse kinematic (IK) is responsible for calculating the angle of each motor (joint) of a kinematic chain, in order to move the end effector from a start point A to an endpoint B.

To solve this problem, this IK component uses the Levenberg-Marquardt algorithm proposed in the article "SBA: A Software Package for Generic Sparse Bundle Adjustment" by Lourakis and Argyros:

    Input: A vector functon f: R^m → R^n with n≥m, a measurement vector x ∈ R^n and an initial parameters estimate p_0 ∈ R^m.
    Output: A vector p+ ∈ R^m minimizing ||x-f(p)||^2.
    Algorithm:
        k:=0;                 v:=2;                     p:=p0;
        A:=transposed(J)·J;   error:=x-f(p);            g:=transposed(J)·error;
        stop:=(||g||∞ ≤ ε1);  μ:=t*max_i=1,...,m (Aii)
        
        while(!stop) and (k<k_max)
             k:=k+1;
             repeat
                   SOLVE (A+μ·I)·δ_p=g;
                   if(||δ_p||≤ ε2·(||p||+ε2))
                        stop:=true;
                   else
                        p_new:=p+δ_p
                        ρ:=(||error||^2-||x-f(p_new)||^2)/(transposed(δ_p)·(μ·δ_p+g));
                        if ρ>0
                            stop:=(||error||-||x-f(p_new)||<ε4·||error||);
                            p:=p_new;
                            A:=transposed(J)·J;    error:=x-f(p);    g:=transposed(J)·error;
                            stop:=(stop) or (||g||∞ ≤ ε1);
                            μ:=μ*max(1/3, 1-(2·ρ-1)^3);
                            v:=2;
                        else
                            μ:=μ*v;
                            v:=2*v;
                        endif
                   endif
             until(ρ>0) or (stop)
             stop:=(||error||≤ ε3);
        endwhile
        p+:=p;
        
This component can resolve three different types of targets:

1. POSE6D: It is the typical target with with translations and rotations in the X, Y and Z axis. The end effector has to be positioned at coordinates (tx, ty, tz) of the target and align their rotation axes with the target, specified in (rx, ry, rz).
2. ADVANCEAXIS: its goal is to move the end effector of the robot along a vector. This is useful for improving the outcome of the above problem, for example, imagine that the hand has been a bit away from a mug. With this feature we can calculate the error vector between the end effector and the mug, and move the effector along the space to place it in an optimal position, near the mug.
3. ALIGNAXIS: Its goal is that the end effector is pointing to target without moving to it but rotated as the target. It may be useful in certain cases where we are more interested in oriented the end effector with the same rotation of the target.

##New component for inverse kinematics

This is the new version for the BIK component, that uses a new interface (before it used BodyInverseKinematics.idsl, now it uses InverseKinematics.idsl) with less useless method and more information. These are the new method of the interface:

1. `TargetState getTargetState (string bodyPart, int targetID)`: This method returns in a 'TargetState` variable the state of a completed target. It has all the information that inverse kinematics calculated on the target (end state, the final angles of the motors, and translational and rotational errors). You only must indicate the part of the robot that executed the target and the identifier of the target.
2. `int setTargetPose6D (string bodyPart, Pose6D target, WeightVector weights)throws IKException`: This method stores a new POSE6D target into the correspondig part of the robot and returns the identifier of the target. You must indicate the part of the robot that will execute the target, the pose of the target (`tx, ty, tz, rx, ry, rz`) and the weights of the translation and rotation coordinates. If the part name doesn't exit, this method throws an exception.
3. `int setTargetAlignaxis (string bodyPart, Pose6D target, Axis ax) throws IKException`: This method stores a new ALIGNAXIS target into the correspondig part of the robot and returns the identifier of the target. You must indicate the part of the robot that will execute the target, the pose of the target (`tx, ty, tz, rx, ry, rz`) and the AXES vector with which we align ourselves. If the part name doesn't exit, this method throws an exception.
4. `int setTargetAdvanceAxis (string bodyPart, Axis ax, float dist) throws IKException`: this method stores a new ADVANCEAXIS target into the correspondig part of the robot and returns the identifier of the target. You must indicate the part of the robot that will execute the target, the axis along we advance and the distance. If the part name doesn't exit, this method throws an exception.
5. `bool getPartState (string bodyPart) throws IKException`: this method returns the state of one part of the robot (if it has targets into his queue of targets or not). If the part name doesn't exit, this method throws an exception.
6. `void goHome (string bodyPart) throws IKException`: this methid moves the part of the robot to the home position of the motors. If the part name doesn't exit, this method throws an exception.
7. `void stop (string bodyPart) throws IKException`; this method stops the movement of the part of the robot and reset the queue of the targets. If the part name doesn't exit, this method throws an exception.

The operation of this new component is simple, you only have to tell the component some important things:

1) the robot whicht will calculate the IK, for example, Ursus. You must put the path of the robot.xml file

    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml

2) the chains that the robot have, for example, 

    LEFTARM
    RIGHTARM
    HEAD

3) the motors that make up the chains, for example,

    LEFTARM=leftShoulder1;leftShoulder2;leftShoulder3;leftElbow;leftForeArm;leftWrist1;leftWrist2

4) And, finally, the tip or end effector of the chain, for example, 
    
    LEFTTIP=grabPositionHandL.

When you send a target position (with his traslation and rotation), you must indicates the chain that will run the target, and the target traslations and rotations weights. 

This revision of the component includes some new enhancements such as:

1. Executes more than once a target. The inverse kinematic result is not the same if the start point of the effector is the robot's home or a point B near tho the goal point.
2. Executes the traslations without the motors of the wrisht (only for Ursus). This makes possible to move the arm with stiff wrist, and then we can rotate easely the wrist when the end effectos is near the target.
3. The new cinverseKInematics component don't move the arm.


##Configuration parameters
We can connect this component with other components through the port 10220.

Like all the components of Robocomp, inversekinematics needs a configuration file to start. You can see one example in

    etc/configDefinitivo

We can find there the following lines:

    BodyInverseKinematics.Endpoints=tcp -p 10220 								#The port of the component
    CommonBehavior.Endpoints=tcp -p 12207
    JoystickAdapterTopic=tcp -p 12226
    
    JointMotorProxy = jointmotor:tcp -p 20000 									#We need the ursuscommonjoint in
                                                                                #order to move the motors of the robot.
    DifferentialRobotProxy = differentialrobot:tcp  -p 10004 -h localhost		#This is the base of the robot
    OmniRobotProxy = omnirobot:tcp  -p 12238 -h localhost							
    InnerModelManagerProxy = innermodelmanager:tcp  -p 11175 -h localhost		#To manage the InnerModel
    
    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml	#A model of the robot Ursus

    #Motors of the robot Ursus:
	RIGHTARM=rightShoulder1;rightShoulder2;rightShoulder3;rightElbow;rightForeArm;rightWrist1;rightWrist2
	RIGHTTIP=grabPositionHandR
	LEFTARM=leftShoulder1;leftShoulder2;leftShoulder3;leftElbow;leftForeArm;leftWrist1;leftWrist2
	LETFTIP=grabPositionHandL
	HEAD=head_yaw_joint;head_pitch_joint
	HEADTIP=rgbd_transform
	
	TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
	
	Ice.Warn.Connections=0
	Ice.Trace.Network=0
	Ice.Trace.Protocol=0
	Ice.ACM.Client=10
	Ice.ACM.Server=10

    
##Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd /robocomp/components/robocomp-ursus/components/inversekinematics
    cp etc/config ourConfig
    
After editing the new config file (ourConfig) we can run the component:

    ./bin/inversekinematics ourConfig
    
It's not necessary put `--Ice.Config=`.