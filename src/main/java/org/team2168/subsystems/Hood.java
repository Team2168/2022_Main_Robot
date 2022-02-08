// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Hood extends SubsystemBase implements Loggable {
  /** Creates a new Hood. */

  public enum HoodPosition {
    TEST,
    TEST1,
    TEST2,
    TEST3
  }

  static Hood instance = null;

  private static WPI_TalonFX hoodMotor = new WPI_TalonFX(Constants.CANDevices.HOOD_MOTOR);

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 73.0/1.0;
  private static final double MAX_RAISED_POSITION_TICKS = 56000;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static boolean kSensorPhase = false;
  private static TalonFXInvertType kMotorInvert = TalonFXInvertType.CounterClockwise; // direction of output shaft rotation when looking at

  // Gains
  private static final double kP = 0.5;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;
  private static final double kArbitraryFeedForward = 0.02;
  private static final int kIzone = 0;
  private static final double kPeakOutput = 1.0;
  private static final double NEUTRAL_DEADBAND = 0.01;
  private static final double ACCELERATION_LIMIT = 35000; // TODO: Change when mechanism is avaialble
  private static final double CRUISE_VELOCITY_LIMIT = 30000; // TODO: Change when mechanism is avaialble
  // private static final int S_CURVE_STRENGTH = 0; // determines the shape of the motion magic graph

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; // amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.2; // s

  private Hood() {
    hoodMotor.configFactoryDefault();
    hoodMotor.configNeutralDeadband(NEUTRAL_DEADBAND);

    hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    hoodMotor.setSensorPhase(kSensorPhase);
    hoodMotor.setInverted(kMotorInvert);

    hoodMotor.configNominalOutputForward(0, kTimeoutMs);
    hoodMotor.configNominalOutputReverse(0, kTimeoutMs);
    hoodMotor.configPeakOutputForward(kPeakOutput, kTimeoutMs);
    hoodMotor.configPeakOutputReverse(-kPeakOutput, kTimeoutMs);

    hoodMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    hoodMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    hoodMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    hoodMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    hoodMotor.configMotionAcceleration(ACCELERATION_LIMIT);
    hoodMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_LIMIT);
    hoodMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    //Don't drive through ends of travel
    hoodMotor.configForwardSoftLimitEnable(true);
    hoodMotor.configForwardSoftLimitThreshold(MAX_RAISED_POSITION_TICKS);
    hoodMotor.configReverseSoftLimitEnable(true);
    hoodMotor.configReverseSoftLimitThreshold(0);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
        CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    hoodMotor.configSupplyCurrentLimit(talonCurrentLimit);
  }

  public static Hood getInstance() {
    if (instance == null) {
      instance = new Hood();
    }
    return instance;
  }

  /**
   * 
   * @return the leader motor position in native ticks
   */
  public double getEncoderTicks() {
    return hoodMotor.getSelectedSensorPosition(kPIDLoopIdx);
  }

  /**
   * Sets the internal sensors of Falcon 500 back to 0.
   * Used when the hood is at a zero position.
   */
  public void setEncoderPosZero() {
    hoodMotor.setSelectedSensorPosition(0.0);
  }

  /**
   * 
   * @param ticks raw F500 motor position
   * @return hood position in degrees relative to is lowered position (0.0)
   */
  public double ticksToDegrees(double ticks) {
    return (ticks / TICKS_PER_REV) / GEAR_RATIO * 360.0;
  }

  /**
   * 
   * @param degrees the hood position in degrees relative to its lowered position (0.0)
   * @return raw F500 motor position
   */
  public double degreesToTicks(double degrees) {
    return (degrees / 360.0) * GEAR_RATIO * TICKS_PER_REV;
  }

  /**
   * Commands the hood to a specied position relative to the zero position.
   * 
   * @param degrees the amount of degrees/angles to move the hood to, positive up.
   */
  public void setPosition(double degrees) {
    hoodMotor.set(ControlMode.MotionMagic, degreesToTicks(degrees),
        DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  /**
   * 
   * @return the hood posiiton in degrees relative to is lowered position (0.0)
   */
  @Log(name = "Position (deg)", rowIndex = 3, columnIndex = 0)
  public double getPositionDegrees() {
    return ticksToDegrees(getPositionTicks());
  }

  /**
   * 
   * @return the hood posititon in raw f500 encoder ticks
   */
  @Log(name = "Position (ticks)", rowIndex = 3, columnIndex = 1)
  public double getPositionTicks() {
    return hoodMotor.getSelectedSensorPosition();
  }

  /**
   * Command the motor in open loop mode.
   * 
   * @param speed percentage of bus voltage to output 1.0 to -1.0
   */
  public void setPercentOutput(double speed) {
    hoodMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  /**
   * Change all motors to their default mix of brake/coast modes.
   * Should be used for normal match play.
   */
  public void setMotorBrake() {
    hoodMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Change all the drivetrain motor controllers to coast mode.
   * Useful for allowing robot to be manually pushed around the field.
   */
  public void setMotorCoast() {
    hoodMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
