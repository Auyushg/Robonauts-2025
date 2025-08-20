
function CoralLoopFloorWrapper()
    CoralLoopFloorBody(0)
end

local status, err = pcall(CoralLoopFloorWrapper)
if not status then
    print ("auton error:",err)
    macro_control:printAdvisory ("auton error: " .. err)
end
