/*******************************************************************************
 *
 * File: LiftControl.cpp - Lift for 2025 Reefscape, includes m_elevator and wrist
 *
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/

#include "gsu/Advisory.h"
#include "FlightBase/LuaState.h"
#include "LiftControl.h"
#include "frc/Preferences.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "Executive/ExecutiveOutputs.h"
#include "RobonautsLibrary/StateData2.h"
#include "FlightBase/RSubsystem.h"


/* Create and initialize all of the elements of the subsystem */
LiftControl::LiftControl(std::string ctrl_name, int gyro_dev_num, std::string can_bus)
    : RSubsystem(ctrl_name)
    , m_pigeon(gyro_dev_num, can_bus)
    , m_pigeon_sim(m_pigeon.GetSimState())
{
    Advisory::pinfo("========================= Creating SubSystem [%s] =========================\n", ctrl_name.c_str());
    addRSpeedController("elevator", &m_elevator);
    addRSpeedController("armPivot", &m_armPivot);
    addOI(&handoff);
    addOI(&transport);
    addOI(&coral_station);
    addOI(&L2);
    addOI(&L3);
    addOI(&L4);
    addOI(&m_lift_spit_button);
    addOI(&m_lift_prepare_to_climb);
    addOI(&processor);
    addOI(&net);
    addOI(&algae_handoff);
    addOI(&m_algae_low_btn);
    addOI(&m_algae_high_btn);

    addOI(&m_el_up_btn);
    addOI(&m_el_dn_btn);
    addOI(&m_arm_up_btn);
    addOI(&m_arm_dn_btn);
    addOI(&m_elev_crt_data_collect);  // test mode
    addOI(&m_arm_crt_data_collect);   // test mode
    addOI(&m_stptUp);
    addOI(&m_stptDown);
    addOI(&m_stptArmUp);
    addOI(&m_stptArmDown);
    addOI(&m_stptElevUp);
    addOI(&m_stptElevDown);
    addOI(&m_lift_brake_btn);    // force to motinitially for checkout, maybe later for offsets
    addOI(&m_lift_coast_btn);    // initially for checkout, maybe later for offsets
    addOI(&m_lift_coast_toggle_btn);    // initially for checkout, maybe later for offsets

    addOI(&m_enable_handoff);
    addOI(&m_disable_handoff);
    // Add gyro for homing arm

}

LiftControl::~LiftControl(void) {}

void LiftControl::initPreferences() {}

void LiftControl::addCancoder(std::string which_joint, int cancoderID, std::string canbus)
{
    if(which_joint == "elevator" || which_joint == "Elevator")
    {
        m_cc_bus_elev = canbus;
        m_cc_id_elev = cancoderID;
    }
    else if(which_joint == "arm" || which_joint == "Arm")
    {
        m_cc_bus_arm = canbus;
        m_cc_id_arm = cancoderID;
    }
}

/* Take care of any initialization that needs to be done after all controls have been created. */
void LiftControl::RobotInit()
{
    // These should get pushed down into the LiftMotor class
    if (m_elevator)
    {
        m_elevator->SetControlMode(RSpeedController::POSITION);
    }
    if (m_armPivot)
    {
        m_armPivot->SetControlMode(RSpeedController::POSITION);

    }
}
void LiftControl::createLiftMotors(double teeth_a_elev, double teeth_b_elev, int rot_elev, double teeth_a_arm, double teeth_b_arm, int rot_arm, double arm_gr)
{
    m_arm_gr = arm_gr;
    armMotor = new LiftMotor("Arm", m_armPivot, m_cc_id_arm, m_cc_bus_arm, teeth_a_arm/teeth_b_arm, 1.0, rot_arm);
    if(armMotor != nullptr)
    {
        // read gyro for homing
        double gyro = m_pigeon.GetRoll(true).GetValueAsDouble();
        if (gyro < -135)
        {
            m_arm_angle_gyro = gyro + 360.0 + m_arm_offset;  // mounted to roll (true requests a refresh)
            Advisory::pinfo("Homing arm with rollover %f = %f + 360 + %f", m_arm_angle_gyro, gyro, m_arm_offset);
        }
        else
        {
            Advisory::pinfo("Homing arm no rollover %f = %f + %f", m_arm_angle_gyro, gyro, m_arm_offset);
            m_arm_angle_gyro = gyro + m_arm_offset;  // mounted to roll (true requests a refresh)
        }
        armMotor->Home(m_arm_gr * m_arm_angle_gyro);
        armMotor->createCrtV2(teeth_a_arm, teeth_b_arm);

    }
    elevatorMotor = new LiftMotor("Elevator", m_elevator, m_cc_id_elev, m_cc_bus_elev, teeth_a_elev/teeth_b_elev, 1.0, rot_elev);
    if(elevatorMotor != nullptr)
    {
        elevatorMotor->createCrtV2(teeth_a_elev, teeth_b_elev);
    }
}

// COMMENT: Should this go away now that we have liftmotor class?
double LiftControl::getCancoderAngle()
{
    double cc = 0.0;
    if(armMotor != nullptr)
    {
        cc = armMotor->getCancoderPosition();
    }
    return cc;
}

void LiftControl::readPreferences()
{
}

// COMMENT: Should this go away now that we have liftmotor class?
double LiftControl::getPositionElevator() // units deg & inches
{
    double m_position = 0.004;
    if (m_elevator != nullptr)
    {
        m_position = m_elevator->GetPosition();
    }
    return (m_position);
}

// COMMENT: Should this go away now that we have liftmotor class?
double LiftControl::getAngleArm() // units deg & inches
{
    double pos = 0.0;
    if(armMotor != nullptr)
    {
        pos = armMotor->getPosition();
    }
    return pos;
}


//COMMENT WE SHOULD HAVE THE ABILITY TO SET LIMITS ON ELEVATOR AS WELL
void LiftControl::setPositionLimitsArm(double min, double max){
    armMotor->setPositionLimits(min, max);
}

// 1. the parameters for this should be just the max value
// 2. it should pass that value down to liftmotor class with this->GetPeriod(), which should be 0.02
// 2.5 the max value might be received as limit, the period might be received as dt
// 3. in lift motor, we need a target position value (the goal) and a command value (the rate limited value)
// 4. it looks something like cmd = RobotUtil::rateLimit(limit, target, cmd, dt);
// 5. in the future, each setpoint might have both a position and a velocity

// THIS SHOULD BE DELETE IN FAVOR OF MORE GENERIC SET VELOCITY LIMITS
void LiftControl::setVelocityLimitArm(double maxVel){
    armMotor->setVelocityLimit(maxVel);
}

void LiftControl::setVelocityLimits(double elev_vel_limit, double arm_vel_limit){
    elevVel = elev_vel_limit;
    armVel = arm_vel_limit;
    elevatorMotor->setVelocityLimit(elev_vel_limit);
    armMotor->setVelocityLimit(arm_vel_limit);
}

/**********************************************************************
 *
 * Runs on a clock, separate from main class at a period specified in RoboControl.lua
 * 1. Read any sensors or joysticks
 * 2. Run logic based on sensor and user inputs (coming in through setAnalog/setDigital)
 * 3. Write to effectors (either motors or relays ...)
 *
 **********************************************************************/
void LiftControl::RobotPeriodic(void)
{
    ReadSensors();
    ReadExecutive();
    if (!IsAutonomous())
    {
        HandleOI();
    }
    writeEffectors();
    StateData2::GetInstance().set<>(name + "/moving", m_moving);
    StateData2::GetInstance().set<>(name + "/elevator_position", elevatorMotor->getPosition());
    StateData2::GetInstance().set<>(name + "/arm_position", armMotor->getPosition());
}

void LiftControl::addSetpointToMap(std::string name, double elevatorPosition, double armPivot, double elevatorVelocity, double armVelocity)
{
    Setpoint Setpoint1;
    Setpoint1.arm_position = armPivot;
    Setpoint1.elevator_position = elevatorPosition;
    Setpoint1.elevator_velocity = elevatorVelocity;
    Setpoint1.arm_velocity = armVelocity;
    setpointMap[name] = Setpoint1;
}

void LiftControl::activateSetpoint(std::string setpointName)
{
    m_currStpt = setpointName;
    armPositionCommand = setpointMap[setpointName].arm_position;
    elevPositionCommand = setpointMap[setpointName].elevator_position;
    armVel = setpointMap[setpointName].arm_velocity;
    elevVel = setpointMap[setpointName].elevator_velocity;


}

std::string LiftControl::getSetpoint(){
    return m_currStpt;
}
void LiftControl::setTotalModifierValues(double elevatorValue, double armValue){
    m_elevStptModifier = elevatorValue;
    m_armStptModifier = armValue;
}

void LiftControl::modifySetpoint(double armValue, double elevatorValue){
    //Advisory::pinfo("Temporarily modifying setpoint %s New elevator position: %f    New Arm Position: %f", m_currStpt.c_str(), elevatorValue, armValue);
    setpointMap[getSetpoint()].arm_position += armValue;
    setpointMap[getSetpoint()].elevator_position += elevatorValue;
    activateSetpoint(getSetpoint());
    writeEffectors();

}
void LiftControl::modifySetpointArm(double armValue){
    //Advisory::pinfo("Temporarily modifying setpoint %s New elevator position: %f    New Arm Position: %f", m_currStpt.c_str(), elevatorValue, armValue);
    setpointMap[getSetpoint()].arm_position += armValue;
    activateSetpoint(getSetpoint());
    writeEffectors();

}
void LiftControl::modifySetpointElevator(double elevatorValue){
    //Advisory::pinfo("Temporarily modifying setpoint %s New elevator position: %f    New Arm Position: %f", m_currStpt.c_str(), elevatorValue, armValue);
    setpointMap[getSetpoint()].elevator_position += elevatorValue;
    activateSetpoint(getSetpoint());
    writeEffectors();

}

void LiftControl::writeEffectors()
{
    m_moving = false;

    if(armMotor != nullptr)
    {
        m_moving |= armMotor->updateCommand(armPositionCommand, GetPeriod().value(), armVel);
    }
    if(elevatorMotor)
    {
        m_moving |= elevatorMotor->updateCommand(elevPositionCommand, GetPeriod().value(), elevVel);
    }
}

//COMMENT IS THIS NEEDED NOW THAT 
void LiftControl::calculateCRT()
{
    output = armMotor->calculateCRT();
    output = elevatorMotor->calculateCRT();
}

void LiftControl::HandleOI()
{
    if (processor.GetButtonPressed())
    {
        activateSetpoint("Processor");
    }
    if (net.GetButtonPressed())
    {
        activateSetpoint("Net");
    }
    if (algae_handoff.GetButtonPressed())
    {
        activateSetpoint("Algae_Handoff");
    }
    if (coral_station.GetButtonPressed())
    {
        activateSetpoint("CoralStation");
    }
    if (handoff.GetButtonPressed())
    {
        if (handoff_enabled){
            activateSetpoint("Handoff"); // set the motor command to 50%
        }
    }
    if (m_enable_handoff.GetButtonReleased()) {
        handoff_enabled = true;
    }
    if (m_disable_handoff.GetButtonPressed()) {
        handoff_enabled = false;
    }
    if (transport.GetButtonPressed())
    {
        activateSetpoint("Transport"); // set the motor command to 50%
    }
    if (L2.GetButtonPressed())
    {
        activateSetpoint("L2"); // set the motor command to 50%
    }
    if (L3.GetButtonPressed())
    {
        activateSetpoint("L3"); // set the motor command to 50%
    }
    if (L4.GetButtonPressed())
    {
        activateSetpoint("L4"); // set the motor command to 50%
    }
    if (m_algae_high_btn.GetButtonPressed())  // high algae
    {
        activateSetpoint("AlgaeHigh");
    }
    if (m_algae_low_btn.GetButtonPressed())  // low algae
    {
        activateSetpoint("AlgaeLow");
    }
    if( m_lift_prepare_to_climb.GetButtonPressed() ){
        m_climbing = true;
    }
    if( m_lift_spit_button.GetButtonReleased()){
        if ( m_currStpt == "L4" ) {
            activateSetpoint("L4Clear");
        }
    }

    if(m_climbing){
        activateSetpoint("Climb");
    }

    // ***************************************************************************
    // BRAKE/COAST MODE AREA

    // toggle the brake/coast
    if (IsDisabled() == true && m_lift_coast_toggle_btn.GetButtonPressed())
    {
        armMotor->setBrakeMode(false);
        elevatorMotor->setBrakeMode(false);
    }
    else if(IsDisabled() == true && m_lift_coast_toggle_btn.GetButtonReleased())
    {
        armMotor->setBrakeMode(true);
        elevatorMotor->setBrakeMode(true);
    }
    if (IsDisabled() == true && m_lift_coast_btn.GetButton())
    {
        armMotor->setBrakeMode(true);
        elevatorMotor->setBrakeMode(true);
    }
    if (IsDisabled() == true && m_lift_brake_btn.GetButton())
    {
        armMotor->setBrakeMode(true);
        elevatorMotor->setBrakeMode(true);
    }
    // END BRAKE/COAST MODE AREA
    // ***************************************************************************


    // test code for bumping up and down
    if(m_el_dn_btn.GetButtonPressed() == true)
    {
        elevPositionCommand -= m_elev_delta;
    }
    if(m_el_up_btn.GetButtonPressed() == true)
    {
        elevPositionCommand += m_elev_delta;
    }
    if(m_arm_dn_btn.GetButtonPressed() == true)
    {
        armPositionCommand -= m_arm_delta;
    }
    if(m_arm_up_btn.GetButtonPressed() == true)
    {
        armPositionCommand += m_arm_delta;
    }
    if (m_stptUp.GetButtonPressed())
    {
        modifySetpoint(m_elevStptModifier, m_armStptModifier);
    }
    if (m_stptDown.GetButtonPressed())
    {
        modifySetpoint(-m_elevStptModifier, -m_armStptModifier);
    }
    if (m_stptArmUp.GetButtonPressed())
    {
        modifySetpointArm(m_armStptModifier);
    }
    if (m_stptArmDown.GetButtonPressed())
    {
        modifySetpointArm(-m_armStptModifier);
    }
    if (m_stptElevUp.GetButtonPressed())
    {
        modifySetpointElevator(m_elevStptModifier);
    }
    if (m_stptElevDown.GetButtonPressed())
    {
        modifySetpointElevator(-m_elevStptModifier);
    }

}

void LiftControl::ReadSensors()
{
    if(elevatorMotor != nullptr)
    {
        elevatorMotor->readSensors();
    }
    if(armMotor != nullptr)
    {
        armMotor->readSensors();
    }
    // read gyro for homing
    m_arm_angle_gyro = m_pigeon.GetRoll().GetValueAsDouble() + m_arm_offset;  // mounted to roll
}

/* Set the robot into a safe mode */
void LiftControl::ReadExecutive(void)
{
    ExecutiveOutputs eo = StateData2::GetInstance().get("executive/outputs", ExecutiveOutputs());

   if(eo.commanding){

    if (eo.lift_disable_handoff){
        handoff_enabled = false;
    }

    activateSetpoint(eo.lift_state);

   }
}

/* Set the robot into a safe mode */
void LiftControl::DisabledInit(void)
{
}

/* Run Disabled specific commands before RobotPeriodic */
void LiftControl::DisabledPeriodic(void)
{
}

/* Prepare for Autonomous Operations */
void LiftControl::AutonomousInit(void)
{
    elevPositionCommand = elevatorMotor->getPosition();
    armPositionCommand = armMotor->getPosition();
    elevatorMotor->initialize();
    armMotor->initialize();
    m_climbing = false;
}

/* Run Autonomous specific commands before RobotPeriodic */
void LiftControl::AutonomousPeriodic(void)
{
}

/* Prepare for Teleop Operations */
void LiftControl::TeleopInit(void)
{
    handoff_enabled = true;
    elevPositionCommand = elevatorMotor->getPosition();
    armPositionCommand = armMotor->getPosition();
    elevatorMotor->initialize();
    armMotor->initialize();
    m_climbing = false;
}

/* Run Teleop specific commands before RobotPeriodic */
void LiftControl::TeleopPeriodic(void) {}

// called when transitioning in to test mode for CRT characterization
void LiftControl::TestInit(void)
{
    elevPositionCommand = elevatorMotor->getPosition();
    armPositionCommand = armMotor->getPosition();
    elevatorMotor->initialize();
    armMotor->initialize();
    m_ctr_dc_running_arm = m_ctr_dc_running_elev = false;
}

// called every cycle before RobotPeriodic when in test mode
void LiftControl::TestPeriodic(void)
{
    double pos=0;
    if(m_elev_crt_data_collect.GetButtonPressed())  // start
    {
        pos = elevatorMotor->getPosition();
        m_ctr_dc_running_elev = true;
        m_ctr_dc_running_arm = false;
        m_ctr_dc_delta = (m_crt_dc_final_pos_elev - pos) / m_crt_dc_steps;
        m_crt_dc_next_time = getPhaseElapsedTime() + m_crt_dc_dt;
        elevPositionCommand = pos;
        m_crt_step = 0;
    }
    if(m_arm_crt_data_collect.GetButtonPressed())  // start
    {
        pos = armMotor->getPosition();
        m_ctr_dc_running_arm = true;
        m_ctr_dc_running_elev = false;
        m_ctr_dc_delta = (m_crt_dc_final_pos_arm - pos) / m_crt_dc_steps;
        armPositionCommand = pos;
        m_crt_dc_next_time = getPhaseElapsedTime() + m_crt_dc_dt;
        m_crt_step = 0;
    }
    RunCrtTest(m_ctr_dc_running_elev, m_ctr_dc_running_arm);

}

void LiftControl::RunCrtTest(bool &running_elev, bool &running_arm)
{
    LiftMotor *mtr = nullptr;
    double *cmd = nullptr;
    double *vel = nullptr;
    bool test = false;
    bool *running = &test;
    double tm;
    if(running_elev == true)  // setup pointers for elevators
    {
        mtr = elevatorMotor;
        cmd = &elevPositionCommand;
        vel = &elevVel;
        running = &running_elev;
    }
    else if(running_arm == true)
    {
        mtr = armMotor;
        cmd = &armPositionCommand;
        vel = &armVel;
        running = &running_arm;
    }
    if(*running == true)  // if something is running, execute
    {
        tm = getPhaseElapsedTime();
        if(tm > m_crt_dc_next_time)
        {
            m_crt_dc_next_time = tm + m_crt_dc_dt;
            *cmd += m_ctr_dc_delta;
            mtr->updateCommand(*cmd, GetPeriod().value(), *vel);
            mtr->printTest();
            m_crt_step++;
            if(m_crt_step >= m_crt_dc_steps)
            {
                *running = false;
            }
        }
    }
}

void LiftControl::setCrtEvalParams(double elev_final, double arm_final, double time_between, int num_steps)
{
    m_crt_dc_final_pos_elev = elev_final;
    m_crt_dc_final_pos_arm = arm_final;
    m_crt_dc_steps = num_steps;
    m_crt_dc_dt = time_between;
}


void LiftControl::addLogVars()
{
    addLogVar("arm/pos", armMotor->m_position);
    addLogVar("arm/tar", armMotor->m_targetStpt);
    addLogVar("arm/cmd", armMotor->m_cmdStpt);
    addLogVar("arm/cur", armMotor->m_current);
    addLogVar("arm/temp", armMotor->m_temperature);
    addLogVar("arm/dc", armMotor->m_dutyCycle);
    addLogVar("arm/rotor", armMotor->m_motorShaftPosition);
    addLogVar("arm/cc", armMotor->m_cancoderPos);
    addLogVar("arm/crt", armMotor->m_crtOutput);
    addLogVar("arm/crt2", armMotor->m_rotor_angle_crt_v2);

    addLogVar("elevator/pos", elevatorMotor->m_position);
    addLogVar("elevator/tar", elevatorMotor->m_targetStpt);
    addLogVar("elevator/cmd", elevatorMotor->m_cmdStpt);
    addLogVar("elevator/cur", elevatorMotor->m_current);
    addLogVar("elevator/temp", elevatorMotor->m_temperature);
    addLogVar("elevator/dc", elevatorMotor->m_dutyCycle);
    addLogVar("elevator/rotor", elevatorMotor->m_motorShaftPosition);
    addLogVar("elevator/cc", elevatorMotor->m_cancoderPos);
    addLogVar("elevator/crt", elevatorMotor->m_crtOutput);
    addLogVar("elevator/crt2", elevatorMotor->m_rotor_angle_crt_v2);
}

void LiftControl::InitSendable(wpi::SendableBuilder &builder)
{
}

/* luaRegister is where to tell Lua which functions to include in RobotControl.lua and autons

   luaRegister has a chained list of calls that define the Subsystem and adds function definitions to Lua.  The line
   that starts "luabridge::getGlobalNamespace" through ".endNamespace()" is technically one line of code.
   To add functions use .addFunction("lua_name",&LiftControl::method) after the "addConstructor" line in the chain.
 */
void luaRegisterLiftControl()
{
    Advisory::pinfo("registering LiftControl with lua");
    lua_State *L = getLuaState();
    luabridge::getGlobalNamespace(L)
     .beginNamespace("robonauts")
      .deriveClass<LiftControl, RSubsystem>("LiftControl")
       .addConstructor<void (*)(std::string,int,std::string)>()
       .addFunction("addSetpointToMap", &LiftControl::addSetpointToMap)
       .addFunction("addCancoder", &LiftControl::addCancoder)
       .addFunction("setPositionLimitsArm", &LiftControl::setPositionLimitsArm)
       .addFunction("setVelocityLimitArm", &LiftControl::setVelocityLimitArm)
       .addFunction("setVelocityLimits", &LiftControl::setVelocityLimits)
       .addFunction("createLiftMotors", &LiftControl::createLiftMotors)
       .addFunction("setCrtEvalParams", &LiftControl::setCrtEvalParams)
       .addFunction("setTotalModifierValues", &LiftControl::setTotalModifierValues)
       .addFunction("activateSetpoint", &LiftControl::activateSetpoint)
      .endClass()
     .endNamespace();
}
