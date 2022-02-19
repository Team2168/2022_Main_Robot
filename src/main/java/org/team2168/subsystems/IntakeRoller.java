// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private static WPI_TalonFX intakeRollerOne = new WPI_TalonFX(CANDevices.INTAKE_MOTOR); 
  private static IntakeRoller instance = null;
  private static TalonFXConfiguration intakeRollerOneConfig = new TalonFXConfiguration();

  private static TalonFXInvertType intakeInvert = TalonFXInvertType.Clockwise;
  
  private static int intakeTimeoutMs = 30;
  private static double peakOutput = 1.0;
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20.0;
  private final double TRIGGER_THRESHOLD_LIMIT = 25;
  private final double TRIGGER_THRESHOLD_TIME = 0.2;


  

  private IntakeRoller() {
    intakeRollerOne.configFactoryDefault();
     intakeRollerOne.setInverted(intakeInvert);

    intakeRollerOne.configNominalOutputForward(0,intakeTimeoutMs);
    intakeRollerOne.configNominalOutputReverse(peakOutput, intakeTimeoutMs);

    intakeRollerOne.configAllSettings(intakeRollerOneConfig);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT,
    TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    intakeRollerOne.configSupplyCurrentLimit(talonCurrentLimit);

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
