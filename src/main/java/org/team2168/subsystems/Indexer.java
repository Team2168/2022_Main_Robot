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

  private double setpointRPM = 0.0;

  private static SupplyCurrentLimitConfiguration indexerCurrentLimit;
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20;
  private static final double TRIGGER_THRESHOLD_LIMIT = 30;
  private static final double TRIGGER_THRESHOLD_TIME = 0.02;

  private static final TalonFXInvertType indexerInvert = TalonFXInvertType.Clockwise;

  private static final double TICKS_PER_REV = 2048.0;
  private static final double TIME_UNITS_VELOCITY_SECS = 0.1;
  private static final double GEAR_RATIO = 2.25/1.0;
  private static final double INCHES_PER_REV = 4.72496;
  private static final double MIN_IN_100_MS = 60.0/0.1;

  //Gains for Velocity
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kF = 0.0;
  
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

    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    motor.config_kF(0, kF);

    motor.configOpenLoopStatusFrameRates();
  }

  public static Indexer getInstance() {
    if (instance == null)
      instance = new Indexer();
    return instance;
  }

  private double inchesToTicks(double inches) {
    return (inches/INCHES_PER_REV) * TICKS_PER_REV * GEAR_RATIO;
  }

  public double getSetpoint() {
    return setpointRPM;
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
   * Commands indexer to velocity Control Mode, then drives the indexer at a set velocity.
   * @param speedRPM the speed in RPM the indexer motor is set to.
   */
  public void driveVelocity(double speedRPM) {
    setpointRPM = speedRPM;
    var setPointVelocity = revs_1min_to_ticks_100ms(speedRPM);
    motor.set(TalonFXControlMode.Velocity, setPointVelocity);
  }

  private double ticks_100ms_to_revs_1min(double ticks) {
    return (ticks/TICKS_PER_REV) * MIN_IN_100_MS * GEAR_RATIO;
  }

  private double revs_1min_to_ticks_100ms(double rotations) {
    return (rotations/MIN_IN_100_MS) * (TICKS_PER_REV/GEAR_RATIO);
  }

  @Log(name = "Speed (RPM)", rowIndex = 3, columnIndex = 1)
  public double getSpeedRotationsPerMinute() {
    return ticks_100ms_to_revs_1min(motor.getSelectedSensorVelocity());
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
