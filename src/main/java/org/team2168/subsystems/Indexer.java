// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final boolean indexer_MOTOR_REVERSED = false;
  private static DigitalInput detector;
  private static Indexer instance = null;
  private static WPI_TalonFX motor;
  private SupplyCurrentLimitConfiguration indexerCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20;
  private final double TRIGGER_THRESHOLD_LIMIT = 30;
  private final double TRIGGER_THRESHOLD_TIME = 0.02;
  TalonFXInvertType indexerInvert = TalonFXInvertType.CounterClockwise;

  private Indexer() {
    detector = new DigitalInput(CANDevices.INDEXER_MOTOR);
    motor.setInverted(indexerInvert);
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }

  public void drive(double speed) {
    motor.set(TalonFXControlMode.PercentOutput, (indexer_MOTOR_REVERSED ? -speed : speed)); {
      speed = speed * -1;
    }
  }

  public int isBallEnteringIndexer() {
    if (!detector.get()) {
      return 1;
    } else {
      return 0;
    }
  }

  public boolean isBallEntering() {
    return !detector.get();
  }

  
  @Override
  public void periodic() {
//  This method will be called once per scheduler run
  }
}
