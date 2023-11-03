function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor,msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.data)
end

function sysCall_init()
    -- Setup for pioneer
    robotHandle=sim.getObject(".")
    sim.setUserParameter(robotHandle,'@enable','')

    leftMotor=sim.getObject("/Pioneer_p3dx_leftMotor")
    rightMotor=sim.getObject("/Pioneer_p3dx_rightMotor")

    -- Use utrasonic sensor and Braitenberg algorithm to control pioneer
    -- usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    -- for i=1,16,1 do
    --     usensors[i]=sim.getObject("/Pioneer_p3dx_ultrasonicSensor"..i)
    -- end

    -- noDetectionDist=0.5
    -- maxDetectionDist=0.2
    -- detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    -- braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    -- braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    -- v0=2
    -- Setup for SICK laser scan
    visionSensor1Handle=sim.getObject("./SICK_TiM310_sensor1")
    visionSensor2Handle=sim.getObject("./SICK_TiM310_sensor2")
    joint1Handle=sim.getObject("./SICK_TiM310_joint1")
    joint2Handle=sim.getObject("./SICK_TiM310_joint2")
    sensorRefHandle=sim.getObject("./SICK_TiM310_ref")

    maxScanDistance=sim.getScriptSimulationParameter(sim.handle_self,'maxScanDistance')
    if maxScanDistance>1000 then maxScanDistance=1000 end
    if maxScanDistance<0.1 then maxScanDistance=0.1 end
    sim.setObjectFloatParam(visionSensor1Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParam(visionSensor2Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    maxScanDistance_=maxScanDistance*0.9999

    scanningAngle=sim.getScriptSimulationParameter(sim.handle_self,'scanAngle')
    --scanningAngle=360
    if scanningAngle>270 then scanningAngle=270 end
    if scanningAngle<2 then scanningAngle=2 end
    scanningAngle=scanningAngle*math.pi/180
    sim.setObjectFloatParam(visionSensor1Handle,sim.visionfloatparam_perspective_angle,scanningAngle/2)
    sim.setObjectFloatParam(visionSensor2Handle,sim.visionfloatparam_perspective_angle,scanningAngle/2)

    sim.setJointPosition(joint1Handle,-scanningAngle/4)
    sim.setJointPosition(joint2Handle,scanningAngle/4)
    red={1,0,0}
    lines=sim.addDrawingObject(sim.drawing_lines,1,0,-1,360,{0,255,0},nil,{10,20,0},nil)
    -- Create needed ROS topic
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
                -- Prepare the sensor publisher and the motor speed subscribers:
        scanPub=simROS.advertise('/scanData', 'std_msgs/Float32MultiArray')
        transformPub=simROS.advertise('/transformData', 'geometry_msgs/Transform')
        leftMotorSub=simROS.subscribe('/leftMotorSpeed','std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/rightMotorSpeed','std_msgs/Float32','setRightMotorVelocity_cb')
    end
   
end
-- Control 2 wheel 
-- function sysCall_actuation()
--     for i=1,16,1 do
--         res,dist=sim.readProximitySensor(usensors[i])
--         if (res>0) and (dist<noDetectionDist) then
--             if (dist<maxDetectionDist) then
--                 dist=maxDetectionDist
--             end
--             detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
--         else
--             detect[i]=0
--         end
--     end
    
--     vLeft=v0
--     vRight=v0
    
--     for i=1,16,1 do
--         vLeft=vLeft+braitenbergL[i]*detect[i]
--         vRight=vRight+braitenbergR[i]*detect[i]
--     end
    
--     sim.setJointTargetVelocity(motorLeft,vLeft)
--     sim.setJointTargetVelocity(motorRight,vRight)
-- end 

function sysCall_sensing()

    measuredData={}
    
    if notFirstHere and simROS then
        -- We skip the very first reading
        sim.addDrawingObjectItem(lines,nil)
        showLines=sim.getScriptSimulationParameter(sim.handle_self,'showLaserSegments')
        r,t1,u1=sim.readVisionSensor(visionSensor1Handle)
        r,t2,u2=sim.readVisionSensor(visionSensor2Handle)
    
        m1=sim.getObjectMatrix(visionSensor1Handle,-1)
        m01=simGetInvertedMatrix(sim.getObjectMatrix(sensorRefHandle,-1))
        m01=sim.multiplyMatrices(m01,m1)
        m2=sim.getObjectMatrix(visionSensor2Handle,-1)
        m02=simGetInvertedMatrix(sim.getObjectMatrix(sensorRefHandle,-1))
        m02=sim.multiplyMatrices(m02,m2)
        if u1 then
            p={0,0,0}
            p=sim.multiplyVector(m1,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u1[2]-1,1 do
                for i=0,u1[1]-1,1 do
                    w=2+4*(j*u1[1]+i)
                    v1=u1[w+1]
                    v2=u1[w+2]
                    v3=u1[w+3]
                    v4=u1[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m01,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m1,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        if u2 then
            p={0,0,0}
            p=sim.multiplyVector(m2,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u2[2]-1,1 do
                for i=0,u2[1]-1,1 do
                    w=2+4*(j*u2[1]+i)
                    v1=u2[w+1]
                    v2=u2[w+2]
                    v3=u2[w+3]
                    v4=u2[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m02,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m2,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        --data=sim.packFloatTable(measuredData)
        --sim.setStringSignal("measuredDataAtThisTime",data)
        simROS.publish(scanPub,{data=measuredData})
        p=sim.getObjectPosition(robotHandle,-1)
        o=sim.getObjectQuaternion(robotHandle,-1)
        simROS.publish(transformPub, {translation={x=p[1],y=p[2],z=p[3]},rotation={x=o[1],y=o[2],z=o[3],w=o[4]}})
    end
    notFirstHere=true
    
    -- measuredData now contains all the points that are closer than the sensor range
    -- For each point there is the x, y and z coordinate (i.e. 3 number for each point)
    -- Coordinates are expressed relative to the sensor frame.
    -- You can access this data from outside via various mechanisms. The best is to first
    -- pack the data, then to send it as a string. For example:
    --
    -- 
    -- data=sim.packFloatTable(measuredData)
    -- sim.setStringSignal("measuredDataAtThisTime",data)
    --
    -- Then in a different location:
    -- data=sim.getStringSignal("measuredDataAtThisTime")
    -- measuredData=sim.unpackFloatTable(data)
    --
    --
    -- Of course you can also send the data via tubes, wireless (sim.tubeOpen, etc., sim.sendData, etc.)
    --
    -- Also, if you send the data via string signals, if you you cannot read the data in each simulation
    -- step, then always append the data to an already existing signal data, e.g.
    --
    -- 
    -- data=sim.packFloatTable(measuredData)
    -- existingData=sim.getStringSignal("measuredDataAtThisTime")
    -- if existingData then
    --     data=existingData..data
    -- end
    -- sim.setStringSignal("measuredDataAtThisTime",data)
end

function sysCall_cleanup()
    sim.removeDrawingObject(lines)
    if simROS then
        -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
        simROS.shutdownPublisher(scanPub)
        simROS.shutdownPublisher(transformPub)
        simROS.shutdownSubscriber(leftMotorSub)
        simROS.shutdownSubscriber(rightMotorSub)
    end
end 
