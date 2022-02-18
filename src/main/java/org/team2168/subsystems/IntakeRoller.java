// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private static WPI_TalonFX intakeRollerOne = new WPI_TalonFX(CANDevices.INTAKE_MOTOR); 
  private static IntakeRoller instance = null;
  private static TalonFXConfiguration intakeRollerOneConfig = new TalonFXConfiguration();
 
  private IntakeRoller() {
    intakeRollerOne.configFactoryDefault();
    intakeRollerOneConfig.supplyCurrLimit.enable = true;
    intakeRollerOneConfig.supplyCurrLimit.currentLimit = 20;
    intakeRollerOneConfig.supplyCurrLimit.triggerThresholdCurrent = 25;
    intakeRollerOneConfig.supplyCurrLimit.triggerThresholdTime = 1;
    intakeRollerOne.configAllSettings(intakeRollerOneConfig);
    }
 public static IntakeRoller getInstance(){
    if (instance == null){
      instance = new IntakeRoller();
    }
    return instance; 
  }

  public void setRollerSpeed(double speed){
    intakeRollerOne.set(speed);
   

    }
 

  
}
