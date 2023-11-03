-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm

function sysCall_init()
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObject("/Pioneer_p3dx_ultrasonicSensor"..i)
    end
    robotHandle=sim.getObject(".")

    motorLeft=sim.getObject("/Pioneer_p3dx_leftMotor")
    motorRight=sim.getObject("/Pioneer_p3dx_rightMotor")
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2
    if simROS then
        transformPub=simROS.advertise('/transformData', 'geometry_msgs/Transform')

end 
-- Control robot following Braitenberg signal

function sysCall_sensing()
    p=sim.getObjectPosition(robotHandle,-1)
    o=sim.getObjectQuaternion(robotHandle,-1)
    simROS.publish(transformPub, {translation={x=p[1],y=p[2],z=p[3]},rotation={x=o[1],y=o[2],z=o[3],w=o[4]}})

function sysCall_actuation()
    for i=1,16,1 do
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
    
    vLeft=v0
    vRight=v0
    
    for i=1,16,1 do
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
    end
    
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
end 


function sysCall_cleanup()
    if simROS then
        -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
        --simROS.shutdownPublisher(dvsPub)
        simROS.shutdownPublisher(transformPub)
        --simROS.shutdownSubscriber(leftMotorSub)
        --simROS.shutdownSubscriber(rightMotorSub)
    end
   
end
