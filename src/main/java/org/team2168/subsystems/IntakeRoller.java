// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import java.net.ConnectException;
import java.sql.Connection;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private static WPI_TalonFX intakeRollerOne; 
  private static IntakeRoller instance; 
  private static TalonFXConfiguration intakeRollerOneConfig;
  private final boolean isIntakeRollerOn;

  public IntakeRoller(final boolean isIntakeRollerOn) {
    intakeRollerOne = new WPI_TalonFX(CANDevices.INTAKE_MOTOR);
    intakeRollerOneConfig = new TalonFXConfiguration();
    this.isIntakeRollerOn = isIntakeRollerOn;


  }



  public static IntakeRoller getInstance(){
    if (instance == null){
      instance = new IntakeRoller(true);
    }
    return instance; 
  }

  public void setRollerSpeed(double speed){
    intakeRollerOne.set(speed);
  }


  public void resetIntakeMotor(double speedTwo){
    intakeRollerOne.set(speedTwo);
    intakeRollerOne.setVoltage(0);
  }
  
  public intakeRollerOn(){
    
  }

  
  
  

  @Override
  public void periodic() {



    
  }
}
