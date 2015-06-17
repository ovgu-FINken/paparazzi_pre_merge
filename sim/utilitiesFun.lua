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

function newIntegerModel()
  
  --ALL DATA IS TO BE REPRESENTED AS INTEGERS
  
  local _DIST_SCALER = 1000
  local _TRIG_SCALER = 1000
  
  local _TOLERABLE_PROXY_DIST = getAsIntScaled(0.70, _DIST_SCALER)
  local _FLIGHT_HEIGHT = getAsIntScaled(1.30, _DIST_SCALER) -- = 1300
  
  local _DEG_TO_RAD_COEFF = getAsIntScaled( (math.pi / 180), _TRIG_SCALER) -- = 174
  local proxyDist = {}	
  local oldProxyDist = {}
  
  local irDist = 0
  local oldIRDist = 0
  
  function sense(sonars, ir, roll, pitch)
    
    for i=1,4 do
      oldProxyDist[i] = proxyDist[i]
    end
    local alpha = getAsIntScaled(pitch, _DEG_TO_RAD_COEFF)
    local beta = getAsIntScaled(roll, _DEG_TO_RAD_COEFF)
    local cosA = intCos(alpha) -- already scaled up by _TRIG_SCALER
    local cosB = intCos(beta)
    -- update x
    proxyDist[1] = getAsIntScaled(sonars[1], _DIST_SCALER) * cosA
    proxyDist[2] = getAsIntScaled(sonars[2], _DIST_SCALER) * cosA
    -- update y
    proxyDist[3] = getAsIntScaled(sonars[3], _DIST_SCALER) * cosB
    proxyDist[4] = getAsIntScaled(sonars[4], _DIST_SCALER) * cosB
    for i=1,4 do
      proxyDist[i] = getAsInt(proxyDist[i]/_TRIG_SCALER) --scale down by _TRIG_SCALER
    end
    
    oldIRDist = irDist
    local tanA = intTan(pitch)
    local tanB = intTan(roll)
    --[[
    formula: realIR = ir / sqrt(1 + tanA_unscaled^2 + tanB_unscaled^2)
    tanA and tanB are scaled by _TRIG_SCALER and ^2 adds one extra scale up
    => scale 1 twice by _TRIG_SCALER
    => sqrt will scale down by _TRIG_SCALER once, but the rest will remain in the denominator
    => multiply the result once by _TRIG_SCALER to eleminate it from the denominator
    --]]
    local irScaled = getAsIntScaled(ir, _DIST_SCALER)
    local scaledAB = _TRIG_SCALER * _TRIG_SCALER + tanA * tanA + tanB * tanB
    local coeff = getAsInt(math.sqrt(scaledAB))
    irDist = getAsInt( irScaled * _TRIG_SCALER / coeff)
  end
  
  function actuate()
    
  end
  --APPROX TRIGONOMETRY--
  -- always give angle in rads
  -- always receive a result which is _TRIG_SCALER times greater than the actual
  function intSin(angle)
    return angle
  end
  
  function intCos(angle)
    --approx function: 1 - (angle^2)/2
    --[[ 
    here: angle is deg * _DEG_TO_RAD_COEFF, but _DEG_TO_RAD_COEFF is already scaled by _TRIG_SCALER
    => angle ^ 2 means we have scaled by _TRIG_SCALER * _TRIG_SCALER
    => we must scale it down once
    => we must also scale 1 up once, e.g we want _TRIG_SCALER * (1 - (angle_not_scaled^2)/2)
    --]]
    local scaled1 = _TRIG_SCALER -- 1 * 1000
    local thi2half = (angle * angle) / (2 * _TRIG_SCALER)
    local diff = scaled1 - getAsInt(thi2half)	
    return diff
  end
  
  function intTan(angle)
    return angle
  end
  --end-of-TRIGONOMETRY
  
  return {
      sense = sense
  }
  
end