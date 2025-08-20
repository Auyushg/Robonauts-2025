/*******************************************************************************
 *
 * File: LiftControl.h - Header file for Lift, which includes elevator and wrist Coral 2025 reefscape
 *
 * Written by:
 * 	The Robonauts
 * 	FRC Team 118
 * 	NASA, Johnson Space Center
 * 	Clear Creek Independent School District
 *
 ******************************************************************************/
#pragma once

#include <iostream>
#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableBuilder.h"
#include "FlightBase/RSubsystem.h"
#include "LiftMotor.h"
#include "RobonautsLibrary/RSpeedController.h"
#include "RobonautsLibrary/OIButton.h"
#include "RobonautsLibrary/OIAxis.h"
#include "RobonautsLibrary/ChineseRemainderTheorem.h"
#include "RobonautsLibrary/Trapezoid.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"

class LiftControl : public RSubsystem, public wpi::Sendable
{
  public:
    LiftControl(std::string name, int gyro_dev_num, std::string can_bus);  // need to resolve why this only calls LiftControl wihtout arguments
    ~LiftControl(void);

    void publish(void);

    void addSetpointToMap(std::string name, double elevatorPosition, double armPivot, double elevatorVelocity, double armVelocity);
    void activateSetpoint(std::string setpointName);

    void armMotorRotationsToDegrees(double conversion);
    void elevatorMotorRotationsToInches(double conversion);

    void addCancoder(std::string which_joint, int cancoderID, std::string canbus);
    void applyConfig();
    double getCancoderAngle();

    void setCurrentLimitArm(double maxValue);
    void setDutyCycleLimitsArm(float min, float max);
    void setPositionLimitsArm(double min, double max);
    void setVelocityLimitArm(double maxVel);
    void setVelocityLimits(double elev_vel_limit, double arm_vel_limit);
    void setBrakeModeArm();

    void createLiftMotors(double ratio_a_elev, double ratio_b_elev, int rot_elev, double ratio_a_arm, double ratio_b_arm, int rot_arm, double arm_gr);
    // setter for configuring crt test code in test mode
    void setCrtEvalParams(double elev_final, double arm_final, double time_between, int num_steps);

    void setTotalModifierValues(double elevatorValue, double armValue);

    std::string getSetpoint();

  protected:
    void RobotInit(void);      // Called once upon creation of control
    void RobotPeriodic(void);  // called every cycle

    void DisabledInit(void);   // called once when system transitions from enabled to disabled
    void DisabledPeriodic(void);   // called every cycle before RobotPeriodic when in disabled

    void AutonomousInit(void); // called once when system transitions from disabled to enabled in autonomous mode
    void AutonomousPeriodic(void); // called every cycle before RobotPeriodic when in autonomous

    void TeleopInit(void);     // called once when system transitions from disabled to enabled in teleop mode
    void TeleopPeriodic(void);     // called every cycle before RobotPeriodic when in teleop

    void TestInit(void);       // called when transitioning in to test mode for CRT characterization
    void TestPeriodic(void);   // called every cycle before RobotPeriodic when in test mode

    void initPreferences();
    void readPreferences();
    void addLogVars();
    void HandleOI();
    void ReadSensors();
    void writeEffectors();
    void ReadExecutive(void); // take inputs from executive
    void modifySetpoint(double armValue, double elevatorValue);
    void modifySetpointArm(double armValue);
    void modifySetpointElevator(double elevatorValue);

    void calculateCRT();

    double getPositionElevator();
    double getAngleArm();

    virtual void InitSendable(wpi::SendableBuilder &builder) override;

  private:
    RSpeedController * m_elevator{nullptr};  // gets created then passed down to lift motor
    RSpeedController * m_armPivot{nullptr};  // gets created then passed down to lift motor
    OIButtonSet processor{"Processor"};
    OIButtonSet net{"Net"};
    OIButtonSet algae_handoff{"Algae_Handoff"};
    OIButtonSet transport{"Transport"};
    OIButtonSet coral_station{"CoralStation"};
    OIButtonSet L2{"L2"};
    OIButtonSet L3{"L3"};
    OIButtonSet L4{"L4"};
    OIButtonSet handoff{"Handoff"};
    OIButtonSet m_lift_spit_button{"LiftSpitButton"};//
    OIButtonSet m_lift_prepare_to_climb{"LiftPrepareToClimb"};

    OIButtonSet m_algae_low_btn{"AlgaeLow"};
    OIButtonSet m_algae_high_btn{"AlgaeHigh"};
    OIButtonSet m_el_up_btn{"el_up"};    // initially for checkout, maybe later for offsets
    OIButtonSet m_el_dn_btn{"el_dn"};      // initially for checkout, maybe later for offsets
    OIButtonSet m_arm_up_btn{"arm_up"};    // initially for checkout, maybe later for offsets
    OIButtonSet m_arm_dn_btn{"arm_dn"};    // initially for checkout, maybe later for offsets
    OIButtonSet m_stptUp{"StptUp"};
    OIButtonSet m_stptDown{"StptDown"};

    OIButtonSet m_stptArmUp{"StptArmUp"};
    OIButtonSet m_stptArmDown{"StptArmDown"};
    OIButtonSet m_stptElevUp{"StptElevUp"};
    OIButtonSet m_stptElevDown{"StptElevDown"};
    OIButtonSet m_lift_brake_btn{"lift_brake"};    // initially for checkout, maybe later for offsets
    OIButtonSet m_lift_coast_btn{"lift_coast"};    // initially for checkout, maybe later for offsets
    OIButtonSet m_lift_coast_toggle_btn{"lift_coast_toggle"};    // initially for checkout, maybe later for offsets

    OIButtonSet m_enable_handoff{"lift_enable_handoff"};
    OIButtonSet m_disable_handoff{"lift_disable_handoff"};

    // begin members for execution of CRT data collection
    OIButtonSet m_elev_crt_data_collect{"collect_elev"};    // data collection for crt
    OIButtonSet m_arm_crt_data_collect{"collect_arm"};    // data collection for crt
    double m_crt_dc_final_pos_elev{28.0};
    double m_crt_dc_final_pos_arm{270.0};  // TODO create Lua interface for this
    double m_ctr_dc_delta{0.0};
    int m_crt_dc_steps{100};
    int m_crt_step{0};
    double m_crt_dc_dt{1.25};
    double m_crt_dc_next_time{0.0};
    bool m_ctr_dc_running_elev{false};
    bool m_ctr_dc_running_arm{false};
    void RunCrtTest(bool &running_elev, bool &running_arm);
    // end members for execution of CRT data collection

    LiftMotor * armMotor{nullptr};
    LiftMotor * elevatorMotor{nullptr};

    double armPositionCommand{0};
    double elevPositionCommand{0};
    double armVel{0};
    double elevVel{0};

    bool m_climbing {false};

    struct Setpoint{
      double elevator_position;
      double arm_position;
      double elevator_velocity;
      double arm_velocity;
    };

    std::map <std::string, Setpoint> setpointMap;

    std::string m_cc_bus_arm{""};
    std::string m_cc_bus_elev{""};
    std::string m_currStpt{""};
    int m_cc_id_arm{1};
    int m_cc_id_elev{2};

    ctre::phoenix6::hardware::CANcoder* m_cancoder_arm{nullptr};
    ctre::phoenix6::hardware::CANcoder* m_cancoder_elevator{nullptr};
    double ratio_A{1.0};
    double ratio_B{1.0};
    double encoder_A{1.0};
    double encoder_B{1.0};

    double output{0.0};
    bool m_moving{false};

    double m_arm_delta{2.5};   // hard code, only for checkout
    double m_elev_delta{0.5};  // hard code, only for checkout

    double m_elevStptModifier{0};
    double m_armStptModifier{0};

    Trapezoid m_elevatorTraj;
    Trapezoid m_armTraj;

  // for pigeon homing
    ctre::phoenix6::hardware::Pigeon2 m_pigeon;
    ctre::phoenix6::sim::Pigeon2SimState & m_pigeon_sim;
    double m_arm_gr{(43.0/46.0 * 15/36 * 1/23.0) / 360.0}; // fix hard code
    double m_arm_offset{90.0};  // zero is straight down, -90 on gyro is straight down
    double m_arm_angle_gyro{0.0};

    bool handoff_enabled{true};
};

void luaRegisterLiftControl();

