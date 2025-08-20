
function CoralLoopFloorBody(pushback)
    macro_control:printAdvisory("starting cat herding")

    if robonauts.MacroLuaControl:IsSimulation() then
        swerve:setPosition(310, 90,180)
--        swerve:setPosition(310, 244,180)
    end
    SetStartLocation()

    reef_select = auton_reef_selector.reef_selection_queue
    --reef_select:AddList("{\"reefs\":[{\"id\":20,\"right\":1,\"level\":4},{\"id\":19,\"right\":0,\"level\":4}]}")

    -- Determine which station to go to depending on starting location
    PursuitPoint = 12;
    if (swerve:getYHalfField() > 158.5) then
        PursuitPoint = 13;
    end

    -- If there is a path loaded check to see if it is compatible to robot start position.
    -- clear the path if it is incompatible
    reef_size = reef_select:GetSize()
    if reef_size > 0 then
        reef = reef_select:Get(0)
        if PursuitPoint == 12 and (reef.id == 20 or reef.id == 11) then
            macro_control:printAdvisory("macro incompatible with starting position")
            reef_select:Clear()
        elseif PursuitPoint == 13 and (reef.id == 22 or reef.id == 9) then
            macro_control:printAdvisory("macro incompatible with starting position")
            reef_select:Clear()
        end
    end

    -- If reef list is empty because none was loaded or path was cleared then add default path
    reef_size = reef_select:GetSize()
    if reef_size == 0 then
        macro_control:printAdvisory("empty reef selection, running default " .. tostring(PursuitPoint))
        if PursuitPoint == 13 then
            reef_select:Add(20,1,4)
            reef_select:Add(18,0,4)
            reef_select:Add(19,1,4)
            reef_select:Add(19,0,4)
            --reef_select:Add(18,1,4)
       else
            reef_select:Add(22,0,4)
            reef_select:Add(18,1,4)
            reef_select:Add(17,0,4)
            reef_select:Add(17,1,4)
            --reef_select:Add(18,1,4)
       end
    end
    reef_size = reef_select:GetSize()

    if pushback == 1 then
        Pushback()
    end

    -- Execute the path
    coral_intake:changeState("trough")
    ee:intakeCoral()
    algae_intake:setStowed()

    for ii=0,reef_size-1,1 do
        reef = reef_select:Get(ii)
        macro_control:printAdvisory("next point " .. tostring(reef.id) .. ", " .. tostring(reef.right) .. ", " .. tostring(PursuitPoint))
        LoopToReef(reef.id, reef.right, PursuitPoint)
        --ee:holdCoral()
        AlignToPole(reef.id, reef.right, reef.level)
        macro_control:printAdvisory("scoring")
        Score()
        coral_intake:changeState("deploy")
        coral_intake:setIntakeCommand(1.0)
    end
    coral_intake:setHotDogCommand(0.5)
    swerve:setRobotCentricMode()
    swerve:drive(-50.0,0,0)
    yield_for(0.25)
    swerve:drive(0.0,0,0)
    coral_intake:changeState("deploy")
    lift:activateSetpoint("Handoff")
    swerve:setFieldOrientedMode()
end

