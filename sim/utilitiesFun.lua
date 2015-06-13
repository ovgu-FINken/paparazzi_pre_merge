function newPIDController(p, i, d, errorScaleCoeff, pidScaleCoeff)
	local integral = 0
	local previousError = 0
	
	
	
	local min = 0
	local max = 0
	local checkMinMax = 0
	local _UPDATE_RATE_COEFF =  getUpdateRateCoeff()
	local timeStep = simGetSimulationTimeStep() * _UPDATE_RATE_COEFF
	
	local pS = getAsIntScaled(p, pidScaleCoeff) -- e.g (int) (5.7*10)
	local iS = getAsIntScaled(i, pidScaleCoeff)
	local dS = getAsIntScaled(d, pidScaleCoeff)
	
	function setMinMax(minParam, maxParam)
	    min = getAsIntScaled(minParam, errorScaleCoeff*pidScaleCoeff)
	    max = getAsIntScaled(maxParam, errorScaleCoeff*pidScaleCoeff)
	    iS = 0 -- in order to avoid integral windup due to clipping effects
	    checkMinMax = 1
	end
	
	function clear()
	    integral = 0
	    previousError = 0
	end
	
	function adjust(error)
		
		local errorS = getAsIntScaled(error, errorScaleCoeff)
		
		integral = integral + (errorS * timeStep)
		local derivative = 0

		if (previousError ~= 0) then
			derivative = (errorS - previousError) / timeStep
		end
		
		previousError = errorS

		local res = (pS * errorS) + (iS * integral) + (dS * derivative)
		
		if(checkMinMax == 1) then -- do we need sigmoid to soften min/max clipping for discrete values?
		  if(res < min) then
		    res = min
		  end
		  if (res > max) then
		    res = max
		  end
		end
		
		return getRealValue(res, errorScaleCoeff, pidScaleCoeff)
	end

	return {
		setMinMax = setMinMax,
		adjust = adjust,
		clear = clear
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
  
  local daPID = newPIDController(distToAngle[1], distToAngle[2], distToAngle[3], getScaleCoeff(), getScalePIDCoeff())
  
  function setMinMax(min, max)
    daPID.setMinMax(min,max)
  end
  
  function adjust(dist)
    return daPID.adjust(minDist - dist)
  end
  
  function clear()
    daPID.clear()
  end
  
  return {
    setMinMax = setMinMax,
    adjust = adjust,
    clear = clear
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
    
    --3) accelaration -> angle: arctan(a/G)	-- not a linear function, performs poorly, not applicable to PID
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

-- SIMULATION ONLY: use to simulate lower update rate (simulated frequency = 20Hz / _UPDATE_RATE_COEFF)
function getUpdateRateCoeff()
  return 5
end

-- use to swap floats with integer arithmetic
function getAsInt(feed)
  return math.floor(feed)
end

-- use to scale floats up/down, in order to maintain some precision when we swap them with integers
function getScaleCoeff()
  return 1000
end

function getScalePIDCoeff()
  return 10
end
  
function getAsIntScaled(feed, scaleCoeff)
  return math.floor(feed * scaleCoeff)
end

function getRealValue(feed, errorScaleCoeff, pidScaleCoeff)
  return feed / (errorScaleCoeff*pidScaleCoeff)
end