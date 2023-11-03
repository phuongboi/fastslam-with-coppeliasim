function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor,msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.data)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSystemTime()
    p=sim.GetObjectPosition(objHandle,relTo)
    o=sim.GetObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function sysCall_init()
    robotHandle=sim.getObject(".")

    leftMotor=sim.getObject("/Pioneer_p3dx_leftMotor") -- Handle of the left motor
    rightMotor=sim.getObject("/Pioneer_p3dx_rightMotor") -- Handle of the right motor
    
    -- Check if the required ROS plugin is there:
 

    -- Prepare DVS handle
    cameraHandle=sim.getObject('/DVS128_sensor')
    angle=sim.getScriptSimulationParameter(sim.handle_self,'cameraAngle')
    if (angle>100) then angle=100 end
    if (angle<34) then angle=34 end
    angle=angle*math.pi/180
    sim.setObjectFloatParam(cameraHandle,sim.visionfloatparam_perspective_angle,angle)
    showConsole=sim.getScriptSimulationParameter(sim.handle_self,'showConsole')
    if (showConsole) then
        auxConsole=sim.auxiliaryConsoleOpen("DVS128 output",500,4)
    end
    showCameraView=sim.getScriptSimulationParameter(sim.handle_self,'showCameraView')
    if (showCameraView) then
        floatingView=sim.floatingViewAdd(0.2,0.8,0.4,0.4,0)
        sim.adjustView(floatingView,cameraHandle,64)
    end

    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
                -- Prepare the sensor publisher and the motor speed subscribers:
        dvsPub=simROS.advertise('/dvsData', 'std_msgs/Int8MultiArray')
        simROS.publisherTreatUInt8ArrayAsString(dvsPub)
        transformPub=simROS.advertise('/transformData', 'geometry_msgs/Transform')
        leftMotorSub=simROS.subscribe('/leftMotorSpeed','std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/rightMotorSpeed','std_msgs/Float32','setRightMotorVelocity_cb')
        resetRobotSub=simROS.subscribe('/resetRobot','std_msgs/Bool','resetRobot_cb')
    end
end


function sysCall_sensing()
    if notFirstHere and simROS then   
    -- Read and formate DVS data at each simulation step
        r,t0,t1=sim.readVisionSensor(cameraHandle)
    
        if (t1) then
            ts=math.floor(sim.getSimulationTime()*1000)
            newData={}
            for i=0,(#t1/3)-1,1 do
                newData[1+i*2]=math.floor(t1[3*i+2])
                newData[2+i*2]=math.floor(t1[3*i+3])
                --newData=newData..string.char(timeStampByte1)
                --newData=newData..string.char(timeStampByte2)

                if (showConsole) then
                    if (t1[3*i+1]>0) then
                        onOff=", on"
                    else
                        onOff=", off"
                    end
                    sim.auxiliaryConsolePrint(auxConsole,"time="..ts.." ms, x="..math.floor(t1[3*i+2])..", y="..math.floor(t1[3*i+3])..onOff.."\n")
                end
            end
        end
        simROS.publish(dvsPub,{data=newData})
        p=sim.getObjectPosition(robotHandle,-1)
        o=sim.getObjectQuaternion(robotHandle,-1)
        simROS.publish(transformPub, {translation={x=p[1],y=p[2],z=p[3]},rotation={x=o[1],y=o[2],z=o[3],w=o[4]}})
    end
    notFirstHere=true
    
    -- newData now contains the same data as would the real sensor (i.e. for each pixel that changed:
    -- 7 bits for the x-coord, 1 bit for polatiry, 7 bits for the y-coord, 1 bit unused, and 1 word for the time stamp (in ms)
    -- You can access this data from outside via various mechanisms. For example:
    --
    -- 
    -- sim.setStringSignal("dataFromThisTimeStep",newData)
    --
    -- Then in a different location:
    -- data=sim.getStringSignal("dataFromThisTimeStep")
    --
    --
    -- Of course you can also send the data via tubes, wireless (sim.tubeOpen, etc., sim.sendData, etc.)
    --
    -- Also, if you you cannot read the data in each simulation
    -- step, then always append the data to an already existing signal data, e.g.
    --
    -- 
    -- existingData=sim.getStringSignal("TheData")
    -- if existingData then
    --     data=existingData..data
    -- end
    -- sim.setStringSignal("TheData",data)
end 

function sysCall_cleanup()
    if simROS then
        -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
        simROS.shutdownPublisher(dvsPub)
        simROS.shutdownPublisher(transformPub)
        simROS.shutdownSubscriber(leftMotorSub)
        simROS.shutdownSubscriber(rightMotorSub)
        simROS.shutdownSubscriber(resetRobotSub)
    end
   
end
