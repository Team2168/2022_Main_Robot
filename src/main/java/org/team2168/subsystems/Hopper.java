// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.Loggable;

public class Hopper extends SubsystemBase implements Loggable {

  private WPI_TalonFX hopperMotor;
  private DigitalInput hopperLineBreak;
  

  private static Hopper instance = null;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;

  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 40; // amps
  private static final double TRIGGER_THRESHOLD_LIMIT = 60; // amp
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s

  public static final double TICKS_PER_REV = 2048.0; 
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;

    public static Hopper getInstance() {
      if (instance == null);
        instance = new Hopper();
      return instance;
    }


  
  /** Creates a new Hopper. */
  private Hopper() {
    hopperMotor = new WPI_TalonFX(CANDevices.HOPPER_MOTOR);
    hopperLineBreak = new DigitalInput(CANDevices.HOPPER_LINE_BREAK);
    

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    hopperMotor.configFactoryDefault();
    hopperMotor.configSupplyCurrentLimit(talonCurrentLimit);
  }

  public void driveHopper(double speed) {
    hopperMotor.set(ControlMode.Velocity, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Log (name = "Ball Is Entering Hopper", rowIndex = 4, columnIndex = 1)
  public boolean isBallEnteringHopper() {
    return !hopperLineBreak.get();
  }

  
}

