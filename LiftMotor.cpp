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
#include "FlightBase/RSubsystem.h"
#include "RobonautsLibrary/RSpeedController.h"
#include "RobonautsLibrary/RobotUtil.h"
#include "RobonautsLibrary/OIButton.h"
#include "RobonautsLibrary/OIAxis.h"
#include "frc/Preferences.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "RobonautsLibrary/ChineseRemainderTheorem.h"
#include "LiftMotor.h"
#include "FlightBase/RSubsystem.h"

LiftMotor::LiftMotor(std::string name, RSpeedController * motor, double cancoderID, std::string canbus, double ratioA, double ratioB, int maximumRotations){

    m_motor = motor;
    m_name = name;

    m_cancoder = new ctre::phoenix6::hardware::CANcoder(cancoderID, canbus);

    motorCRT = new ChineseRemainderTheorem(m_name, ratioA, ratioB, maximumRotations);
#if 0   // testcode for debugging algorithm over smartdashboard/shuffleboard
    frc::SmartDashboard::PutNumber(m_name + "_rotor", 1);
    frc::SmartDashboard::PutNumber(m_name + "_cc", 2);
#endif
#ifdef LIFT_SHUFFLEBOARD
    if(name == "Elevator")
    {
        frc::Shuffleboard::GetTab("LiftControl").Add(name.c_str(), *this).WithSize(3, 4).WithPosition(0, 0);
    }
    else
    {
        frc::Shuffleboard::GetTab("LiftControl").Add(name.c_str(), *this).WithSize(3, 4).WithPosition(3, 0);
    }
#else
    frc::Shuffleboard::GetTab("LiftControl").Add(name.c_str(), *this);
#endif

}

void LiftMotor::setPID(double kp, double ki, double kd){
    m_motor->SetP(kp);
    m_motor->SetI(ki);
    m_motor->SetD(kd);
    m_motor->SetF(0);
}

void LiftMotor::createCrtV2(int teeth_a, int teeth_b)
{
    if(motorCRT == nullptr)
    {
        motorCRT = new ChineseRemainderTheorem(m_name, teeth_a, teeth_b);
    }
    else
    {
        motorCRT->CreateV2(teeth_a, teeth_b);
        if(m_motor && m_cancoder){
            readSensors();
            if(m_name == "Elevator")  // home elevator using CRT
            {
                m_motor->SetPosition(0.0);
            }
        }
    }
}

// Home for arm running of gyro
void LiftMotor::Home(double arm_angle_in_motor_rev)
{
    Advisory::pinfo("create arm: %f", arm_angle_in_motor_rev);

    if(m_name =="Arm")  // home elevator using gyro
    {
        m_motor->SetPosition(arm_angle_in_motor_rev);
    }
    Advisory::pinfo("initializing %s to 0.0", m_name.c_str());
}

void LiftMotor::setVelocityLimit(double maxVel){
    m_maxVelocity = maxVel;
}

void LiftMotor::initialize(){
    m_cmdStpt = m_targetStpt = m_motor->GetPosition();
}

void LiftMotor::setCurrentLimit(double maxValue){
    m_motor->SetCurrentLimit(maxValue);

}

void LiftMotor::setDutyCycleLimits(float min, float max){
    m_motor->SetClosedLoopOutputLimits(0,0, min, max);
}

void LiftMotor::setPositionLimits(double min, double max){
    m_minLimit = min;
    m_maxLimit = max;

}

double LiftMotor::getVelocity(){
    return m_motor->GetSpeed();
}


double LiftMotor::getPosition(){
    return m_position;
}

double LiftMotor::getCancoderPosition(){
    return m_cancoder->GetAbsolutePosition().GetValue().value();
}

void LiftMotor::readSensors(){

    if(m_motor){
        m_position = m_motor->GetPosition();
        m_velocity = m_motor->GetSpeed();
        m_motorShaftPosition = m_motor->GetRotorPosition();
        m_current = m_motor->GetOutputCurrent();
        m_dutyCycle = m_motor->GetMotorOutputPercent();
        m_temperature = m_motor->GetTemperature();
    }
}

void LiftMotor::setBrakeMode(bool value){
    m_motor->SetBrakeMode(value);
    m_motor->ApplyConfig();   // be cautious if sending frequently
}

double LiftMotor::calculateCRT(){

    if(motorCRT != nullptr)
    {
        m_crtOutput = motorCRT->CalculateCRT(m_motorShaftPosition * 360, m_cancoderPos * 360);
        m_rotor_angle_crt_v2 = m_motor->GetOutputPerCount() *  m_crtOutput / 360;
        motorCRT->Update(m_motorShaftPosition * 360, m_cancoderPos * 360);  // update needed
    }
    return m_crtOutput;

}


bool LiftMotor::updateCommand(double value, double period, double velocity){
    m_targetStpt = value;
    m_maxVelocity = velocity;
    m_targetStpt = std::clamp(m_targetStpt, m_minLimit, m_maxLimit);
    m_cmdStpt = RobotUtil::rateLimit(m_maxVelocity, m_targetStpt, m_cmdStpt, period);
    m_motor->Set(m_cmdStpt);
    return (m_cmdStpt != m_targetStpt);
}

// print an advisory when running in test mode
void LiftMotor::printTest()
{
    Advisory::pinfo("%s,%f,%f,%f,%f,%f,%f", m_name.c_str(), m_crtOutput, m_position, m_targetStpt, m_velocity, m_motorShaftPosition, m_cancoderPos);
}

void LiftMotor::InitSendable(wpi::SendableBuilder &builder)
{
#ifdef LIFT_SHUFFLEBOARD
    builder.AddDoubleProperty("01. Pos", [this] { return m_position; }, nullptr);
    builder.AddDoubleProperty("02. Tar", [this] { return m_targetStpt; }, nullptr);
    builder.AddDoubleProperty("03. Cmd", [this] { return m_cmdStpt; }, nullptr);
    builder.AddDoubleProperty("04. Cur", [this] { return m_current; }, nullptr);
    builder.AddDoubleProperty("06. temp", [this] { return m_temperature; }, nullptr);
    builder.AddDoubleProperty("05. DC ", [this] { return m_dutyCycle; }, nullptr);
    builder.AddDoubleProperty("07. rotor", [this] { return m_motorShaftPosition; }, nullptr);
    builder.AddDoubleProperty("08. CC", [this] { return m_cancoderPos; }, nullptr);
    builder.AddDoubleProperty("09. CRT", [this] { return m_crtOutput; }, nullptr);
    builder.AddDoubleProperty("10. CRT2", [this] { return m_rotor_angle_crt_v2; }, nullptr);
#else
    std::string title = m_name + " Position";
    builder.AddDoubleProperty(title, [this] { return m_position; }, nullptr);
#endif
}

