// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private WPI_TalonFX motor;
  private final boolean _INDEXER_MOTOR_REVERSED = false;
  private static DigitalInput detector;
  private static Indexer _instance = null;

  private SupplyCurrentLimitConfiguration indexerCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20;
  private final double TRIGGER_THRESHOLD_LIMIT = 30;
  private final double TRIGGER_THRESHOLD_TIME = 0.02;

  private Indexer() {
    detector = new DigitalInput(CANDevices.INDEXER_MOTOR);
  }

  public static Indexer getInstance() {
    if (_instance == null) {
      _instance = new Indexer();
    }
    return _instance;
  }

  public void drive(double speed) {
    if (_INDEXER_MOTOR_REVERSED) {
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

  // @Override
  // public void periodic() {
  // This method will be called once per scheduler run
  // }
}
