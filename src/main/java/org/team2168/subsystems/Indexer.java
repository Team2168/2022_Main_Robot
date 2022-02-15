// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;
import org.team2168.Constants.DIO;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private static DigitalInput detector;
  private static Indexer instance = null;
  private static WPI_TalonFX motor;
  private static SupplyCurrentLimitConfiguration indexerCurrentLimit;
  private static final boolean indexer_MOTOR_REVERSED = false;
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20;
  private static final double TRIGGER_THRESHOLD_LIMIT = 30;
  private static final double TRIGGER_THRESHOLD_TIME = 0.02;
  private static final TalonFXInvertType indexerInvert = TalonFXInvertType.CounterClockwise;;

  private Indexer() {
    detector = new DigitalInput(DIO.INDEXER_SENSOR);
    TalonFXInvertType indexerInvert = TalonFXInvertType.CounterClockwise;
    motor.setInverted(indexerInvert);
    indexerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, 
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    motor.configSupplyCurrentLimit(indexerCurrentLimit);
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }

  /**
   * 
   * @param speed should be set to between 1.0 and -1.0, depending on if you need it to intake or spit out a ball
   * with 1.0 being to move towards the shooter, and -1.0 away from it.
   */
  public void drive(double speed) {
    motor.set(TalonFXControlMode.PercentOutput, (indexer_MOTOR_REVERSED ? -speed : speed));
  }

  /**
   * 
   * @return boolean is meant to detect the presence of a ball in the indexer
   */
  public boolean isBallPresent() {
    return !detector.get();
  }

  
  @Override
  public void periodic() {
    //  This method will be called once per scheduler run
  }
}
