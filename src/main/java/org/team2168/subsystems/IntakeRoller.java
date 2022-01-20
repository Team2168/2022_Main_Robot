// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private WPI_TalonFX intakeRollerOne; 
  private IntakeRoller _instance; 
  private TalonFXConfiguration intakeRollerOneConfig;


  public IntakeRoller() {
    intakeRollerOne = new WPI_TalonFX(CANDevices.INTAKE_MOTOR);
    intakeRollerOneConfig = new TalonFXConfiguration();

  }


  public IntakeRoller getInstance(){
    if (_instance == null){
      _instance = new IntakeRoller();
    }
    return _instance; 
  }

  public void MotorSpeed(double speed){
    intakeRollerOne.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
