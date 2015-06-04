function newPIDController(p, i, d)
	local integral = 0
	local previousError = 0
	
	
	
	local min = 0
	local max = 0
	local checkMinMax = 0
	
	function setMinMax(minParam, maxParam)
	    min = minParam
	    max = maxParam
	    checkMinMax = 1
	end
	
	function adjust(error)
		local _UPDATE_RATE_COEFF = 5	-- SIMULATION ONLY: use to simulate lower update rate of 4Hz (= 20Hz / _UPDATE_RATE_COEFF)
		local timeStep = simGetSimulationTimeStep()
		timeStep = timeStep * _UPDATE_RATE_COEFF
		integral = integral + (error * timeStep)
		local derivative = (error - previousError) / timeStep

		if (previousError == 0) then
			derivative = 0
		end
		
		previousError = error

		local res = (p * error) + (i * integral) + (d * derivative)
		
		if(checkMinMax == 1) then
		  if(res < min) then
		    res = min
		  end
		  if (res > max) then
		    res = max
		  end
		end
		
		return res
	end

	return {
		setMinMax = setMinMax,
		adjust = adjust
	}
end

--[[SIMULATION ONLY: use this controller for x,y-plane to create simple flight behaviour, so we can test how the zPID controller works--]]
function newLinearController(script, paramName, closeDist, remoteDist, attractionAngle, repulsionAngle)
  function adjust(currentDist)
    if(currentDist < closeDist) then
      simSetScriptSimulationParameter(script, paramName, repulsionAngle)
    elseif(currentDist > remoteDist) then
      simSetScriptSimulationParameter(script, paramName, attractionAngle)
    else
      simSetScriptSimulationParameter(script, paramName, 0)
    end
  end
  return {
    adjust = adjust
  }
end

-- the real controller that we need for the final task
function newDirectDistToAngleController(distToAngle, minDist)
  
  local daPID = newPIDController(distToAngle[1], distToAngle[2], distToAngle[3])
  
  function setMinMax(min, max)
    daPID.setMinMax(min,max)
  end
  
  function adjust(dist)
    return daPID.adjust(minDist - dist)
  end
  
  return {
    setMinMax = setMinMax,
    adjust = adjust
  }
end

--[[ IGNORE -- this controller is not required for now, DO NOT IMPLEMENT IN C --]]
function newDistToAngleController(distToVel, velToAccel, accelToAngle, minDist)
  
  local dvPID = newPIDController(distToVel[1], distToVel[2], distToVel[3])
  local vaPID = newPIDController(velToAccel[1], velToAccel[2], velToAccel[3])
  local aaPID = newPIDController(accelToAngle[1], accelToAngle[2], accelToAngle[3])
  
  local lastVel = 0
  local lastAccel = 0
  
  function setMinMaxDistToVel(min, max)
    dvPID.setMinMax(min, max)
  end
  
  function setMinMaxVelToAccel(min, max)
    vaPID.setMinMax(min, max)
  end
  
  function setMinMaxAccelToAngle(min, max)
    aaPID.setMinMax(min, max)
  end
  
  
  function adjust(currDist, oldDist)
    
    local dt = simGetSimulationTimeStep()
    
    --1) distance -> velocity: dx/dt
    local targetVel = (minDist - currDist) / dt
    local currVel = (currDist - oldDist) / dt
    local errorVel = targetVel - currVel
    
    --2) velocity -> accelaration: dv/dt
    local targetAccel = (dvPID.adjust(errorVel) - currVel) / dt
    local currAccel = (currVel - lastVel) / dt
    local errorAccel = targetAccel - currAccel
    
    --3) accelaration -> angle: arctan(a/G)
    local targetAngle = math.atan(vaPID.adjust(errorAccel) / 9.81)
    local currAngle = math.atan(currAccel / 9.81)
    local errorAngle = targetAngle - currAngle
    
    lastVel = currVel
    lastAccel = currAccel
    
    return aaPID.adjust(errorAngle)
  end
 
  return {
    setMinMaxDistToVel = setMinMaxDistToVel,
    setMinMaxVelToAccel = setMinMaxVelToAccel,
    setMinMaxAccelToAngle = setMinMaxAccelToAngle,
    adjust = adjust
  }
end

