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
#pragma once

#include "gsu/Advisory.h"
#include "FlightBase/LuaState.h"
#include "LiftControl.h"
#include "FlightBase/RSubsystem.h"
#include "RobonautsLibrary/RSpeedController.h"
#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableBuilder.h"
#include "RobonautsLibrary/OIButton.h"
#include "RobonautsLibrary/OIAxis.h"
#include "frc/Preferences.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "RobonautsLibrary/ChineseRemainderTheorem.h"
#include "RobonautsLibrary/RobotUtil.h"
#include <iostream>


class LiftMotor : public wpi::Sendable
{
    public:
        LiftMotor(std::string name, RSpeedController * motor, double cancoderID, std::string canbus, double ratioA, double ratioB, int maximumRotations);

        void setVelocityLimit(double maxVelocity);
        void setCurrentLimit(double maxValue);
        void setDutyCycleLimits(float min, float max);
        void setPositionLimits(double min, double max);

        double getVelocity();
        double getPosition();
        double getCancoderPosition();

        bool updateCommand(double value, double period, double velocity);

        void readSensors();

        double calculateCRT();

        void setBrakeMode(bool value);
        void createCrtV2(int teeth_a, int teeth_b);
        void setPID(double kp, double ki, double kd);
        void initialize();
        void printTest();
        void Home(double arm_angle_motor_rev);

        virtual void InitSendable(wpi::SendableBuilder &builder) override;

    public:
        std::string m_name{""};

        RSpeedController * m_motor{nullptr};
        ctre::phoenix6::hardware::CANcoder* m_cancoder{nullptr};

        double m_position{0};
        double m_velocity{0};
        double m_motorShaftPosition{0};
        double m_current{0};
        double m_dutyCycle{0};
        double m_temperature{0};

        double m_cancoderPos{0};

        double m_cmdStpt{0};
        double m_targetStpt{0};
        double m_crtOutput{0};

        ChineseRemainderTheorem * motorCRT{nullptr};
        // CRT v2 variables
        int m_tooth1{0};
        int m_tooth2{0};
        double m_rotor_angle_crt_v2{0.0};

        double m_minLimit{-100000.0};
        double m_maxLimit{100000.0};  // no pos limits by default
        double m_maxVelocity{1};


};

