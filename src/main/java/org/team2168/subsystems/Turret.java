// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team2168.Constants;
import org.team2168.utils.CanDigitalInput;
import org.team2168.utils.Gains;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private CanDigitalInput hallEffectSensor;
  private TalonFX turretMotor;
  private static Turret instance = null;
  private final int TICKS_PER_REV = 2048;

  //gains
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = true;
  public static boolean kMotorInvert = false;
  static final Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);

  public Turret() {
    turretMotor = new TalonFX(Constants.TALONFX_TURRET_MOTOR);
    hallEffectSensor = new CanDigitalInput(turretMotor);

    turretMotor.configFactoryDefault();

    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    turretMotor.setSensorPhase(kSensorPhase);
    turretMotor.setInverted(kMotorInvert);

    turretMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    turretMotor.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    turretMotor.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    turretMotor.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    turretMotor.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);
  }

  public static Turret getInstance() {
    if(instance == null) 
      instance = new Turret();
    return instance;
  }

  public boolean isTurretAtZero() {
    return hallEffectSensor.isFwdLimitSwitchClosed();
  }

  /**
   * Rotates the turret to a position
   * @param rotation Should be between -1 and 1
   */
  public void rotateTurret(double rotation) {
    turretMotor.set(ControlMode.Position, rotation * TICKS_PER_REV);
  }

  public double getposition() {
    return turretMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
