// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import org.team2168.Constants.CANDevices;
import org.team2168.Constants.DIO;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Indexer extends SubsystemBase implements Loggable {
  /** Creates a new Indexer. */
  private static DigitalInput detector;
  private static TalonFXHelper motor;

  private static SupplyCurrentLimitConfiguration indexerCurrentLimit;
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20;
  private static final double TRIGGER_THRESHOLD_LIMIT = 30;
  private static final double TRIGGER_THRESHOLD_TIME = 0.02;
  private static final TalonFXInvertType indexerInvert = TalonFXInvertType.Clockwise;
  
  private static Indexer instance = null;
  private Indexer() {
    detector = new DigitalInput(DIO.INDEXER_SENSOR);
    motor = new TalonFXHelper(CANDevices.INDEXER_MOTOR);

    motor.configFactoryDefault();
    motor.setInverted(indexerInvert);
    indexerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, 
      CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    motor.configSupplyCurrentLimit(indexerCurrentLimit);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configOpenLoopStatusFrameRates();
  }

  public static Indexer getInstance() {
    if (instance == null)
      instance = new Indexer();
    return instance;
  }

  /**
   * 
   * @param speed should be set to between 1.0 and -1.0, depending on if you need it to intake or spit out a ball
   * with 1.0 being to move towards the shooter, and -1.0 away from it.
   */
  public void drive(double speed) {
    motor.set(TalonFXControlMode.PercentOutput, speed);
  }

  /**
   * 
   * @return boolean is meant to detect the presence of a ball in the indexer
   */
  @Log(name = "Is ball present?")
  public boolean isBallPresent() {
    return !detector.get();
  }

  
  @Override
  public void periodic() {
    //  This method will be called once per scheduler run
  }
}
