// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants;

import io.github.oblarg.oblog.annotations.Log;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  static Hood instance = null;

  private static WPI_TalonFX hoodMotor = new WPI_TalonFX(Constants.CANDevices.CLIMBER_MOTOR_1);

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = (40.0 / 10.0) * (40.0 / 14.0) * (24.0 / 18.0);
  private static final double SPROCKET_RADIUS_INCHES = 0.6589;
  private static final double INCHES_PER_REV = SPROCKET_RADIUS_INCHES * 2 * Math.PI;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static boolean kSensorPhase = false;
  private static TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise; // direction of output shaft rotation
                                                                               // when looking at

  private static final double TIME_UNITS_OF_VELOCITY = 0.1; // in seconds

  // Gains
  private static final double kP = 0.2;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;
  private static final double kArbitraryFeedForward = 0.034;
  private static final int kIzone = 0;
  private static final double kPeakOutput = 1.0;
  private static final double NEUTRAL_DEADBAND = 0.01;
  private static final double ACCELERATION_LIMIT = inchesToTicks(6.0) * TIME_UNITS_OF_VELOCITY; // TODO: Change when mechanism is avaialble
  private static final double CRUISE_VELOCITY_LIMIT = inchesToTicks(12.0) * TIME_UNITS_OF_VELOCITY; // TODO: Change when mechanism is avaialble
  // private static final int S_CURVE_STRENGTH = 0; // determines the shape of the
  // motion magic graph

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

    hoodMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    hoodMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

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
   * @return true when the lift is fully lowered
   */
  @Log(name = "At Zero", rowIndex = 3, columnIndex = 0)
  public boolean isAtZeroPosition() {
    return hoodMotor.isRevLimitSwitchClosed() == 1;
  }

  public boolean isAtUpperPosition() {
    return hoodMotor.isFwdLimitSwitchClosed() == 1;
  }

  // methods that convert a velocity value from seconds to hundreds of milliseconds and back
  private static double convertVelocitySecondstoHundredMs(double speed) {
    return speed * TIME_UNITS_OF_VELOCITY;
  }

  private static double convertVelocityHundredMstoSeconds(double speed) {
    return speed / TIME_UNITS_OF_VELOCITY;
  }

  /**
   * 
   * @return the leader motor position in native ticks
   */
  private double getEncoderTicks() {
    return hoodMotor.getSelectedSensorPosition(kPIDLoopIdx);
  }

  /**
   * Sets the internal sensors of Falcon 500 back to 0.
   * Used when the lift is at a zero position.
   */
  public void setEncoderPosZero() {
    hoodMotor.setSelectedSensorPosition(0.0);
  }

  /**
   * 
   * @param inches the lift position in inches
   * @return position in F500 internal encoder ticks
   */
  private static double inchesToTicks(double inches) {
    return (inches / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  /**
   * 
   * @param ticks the lift position in F500 interal encoder ticks
   * @return the lift position in inches
   */
  private static double ticksToInches(double ticks) {
    return (ticks / TICKS_PER_REV) / GEAR_RATIO * INCHES_PER_REV;
  }

  /**
   * 
   * @return current lift velocity (inches/second)
   */
  @Log(name = "Speed (In-s)", rowIndex = 3, columnIndex = 3)
  public double getSpeedInchesPerSecond() {
    return convertVelocityHundredMstoSeconds(ticksToInches(hoodMotor.getSelectedSensorVelocity()));
  }

  /**
   * 
   * @return current lift position (inches), zero is fully lowered
   */
  @Log(name = "Position (In)", rowIndex = 3, columnIndex = 2)
  public double getPositionInches() {
    return ticksToInches(hoodMotor.getSelectedSensorPosition());
  }

  //
  /**
   * Commands the lift at a specified velocity.
   * 
   * @param speedInInchesPerSec speed to run the lift at, positive up.
   */
  public void setSpeed(double speedInInchesPerSec) {
    hoodMotor.set(ControlMode.Velocity, inchesToTicks(speedInInchesPerSec) * TIME_UNITS_OF_VELOCITY);
  }

  /**
   * Commands the lift to a specied position relative to the zero position.
   * 
   * @param inches the position to move the lift to, positive up.
   */
  public void setPosition(double inches) {
    hoodMotor.set(ControlMode.MotionMagic, inchesToTicks(inches),
        DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  /**
   * Command the motor in open loop mode.
   * 
   * @param speed percentage of bus voltage to output 1.0 to -1.0
   */
  public void setPercentOutput(double speed) {
    hoodMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
