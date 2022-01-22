// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants;
import org.team2168.utils.CanDigitalInput;
import org.team2168.utils.Gains;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private CanDigitalInput hallEffectSensor;
  private WPI_TalonFX turretMotor;
  private static Turret instance = null;

  private final int TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 1.0;
  private final double TICKS_PER_WHEEL_ROTATION = TICKS_PER_REV * GEAR_RATIO;

  private final double TICKS_PER_SECOND = TICKS_PER_WHEEL_ROTATION;
  private final double TICKS_PER_100_MS = TICKS_PER_SECOND / 10;
 
  //About 260/360 degrees
  private final int MAX_ROTATION_TICKS = 1480;

  private final double ACCELERATION = TICKS_PER_100_MS;
  private final double CRUISE_VELOCITY = TICKS_PER_100_MS;

  //gains
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = true;
  public static boolean kMotorInvert = false;

  //                                     P,   I,   D,   F,  I zone, and Peak output
  static final Gains kGains = new Gains(0.5, 0.0, 1.0, 0.0, 0, 1.0);

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amps
  private final double TRIGGER_THRESHOLD_TIME = 0.02; //seconds


  private Turret() {
    turretMotor = new WPI_TalonFX(Constants.CANDevices.TALONFX_TURRET_MOTOR);
    hallEffectSensor = new CanDigitalInput(turretMotor);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    //Configuring the turret motor
    turretMotor.configFactoryDefault();
    turretMotor.configSupplyCurrentLimit(talonCurrentLimit);

    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    turretMotor.setSensorPhase(kSensorPhase);
    turretMotor.setInverted(kMotorInvert);

    turretMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    turretMotor.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    turretMotor.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    turretMotor.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    turretMotor.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);

    turretMotor.configMotionAcceleration(ACCELERATION);
    turretMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
  }

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  public boolean isTurretAtZero() {
    return hallEffectSensor.isFwdLimitSwitchClosed();
  }

  /**
   * Rotates the turret to a position
   * 
   * @param rotation Between -1 and 1, -1 is a -360 degree rotation, 0 is no movement, and 1 is a 360 degree rotation
   */

  public void setRotation(double rotation) {
    turretMotor.set(ControlMode.MotionMagic, (rotation * TICKS_PER_WHEEL_ROTATION) /10);
  }
  //CHECK THIS
  public void setRotationDegrees(double degrees) {
    turretMotor.set(ControlMode.MotionMagic, ((degrees / 360) * TICKS_PER_WHEEL_ROTATION) /10);
  }

  /**
   * Sets the velocity 
   * @param velocity The target velocity in ticks per 100ms
   */
  public void setVelocity(double velocity) {
    turretMotor.set(ControlMode.Velocity, velocity);
  }

  public void drive(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Returns the internal sensor's position
   * @return The internal sensor's position
   */
  public double getEncoderPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  public void zeroEncoder() {
    turretMotor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}