// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private static WPI_TalonFX intakeRollerOne; 
  private static IntakeRoller instance; 
  private static TalonFXConfiguration intakeRollerOneConfig;
  

   private IntakeRoller() {
    intakeRollerOne.configFactoryDefault();
    intakeRollerOneConfig.supplyCurrLimit.enable = true;
    intakeRollerOneConfig.supplyCurrLimit.currentLimit = 20;
    intakeRollerOneConfig.supplyCurrLimit.triggerThresholdCurrent = 25;
    intakeRollerOneConfig.supplyCurrLimit.triggerThresholdTime = 1;
    intakeRollerOne.configAllSettings(intakeRollerOneConfig);
    
    intakeRollerOne = new WPI_TalonFX(CANDevices.INTAKE_MOTOR);
    intakeRollerOneConfig = new TalonFXConfiguration();
  
 }
 public static IntakeRoller getInstance(){
    if (instance == null){
      instance = new IntakeRoller();
    }
    return instance; 
  }

  public void setRollerSpeed(double speed){
    intakeRollerOne.set(speed);
    intakeRollerOne.set(TalonFXControlMode.PercentOutput, speed);

    }
  public void resetIntakeMotor(double speedTwo){
    intakeRollerOne.set(speedTwo);
    intakeRollerOne.setVoltage(speedTwo);
    intakeRollerOne.set(TalonFXControlMode.PercentOutput, speedTwo);
    intakeRollerOne.set(ControlMode.Disabled, 0);
    
  }

  
}
