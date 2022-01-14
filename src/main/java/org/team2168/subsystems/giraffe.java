// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class giraffe extends SubsystemBase {
  /** Creates a new giraffe. */
  public static final int kSlotIdx = 0;
  public final static int kPIDLoopIdx = 0;
  public final static int kTimeoutMs = 30;
  public static final boolean kSensorPhase = true;
  public static final boolean kMotorInvert = false;

  public final double kP = 0.15;
  public final double kI = 0.0;
  public final double kD = 1.0;
  public final double kF = 0.0;
  public final int kIzone = 0;
  public final double kPeatOutput = 1.0;


  public TalonFX talon = new TalonFX(0);
  private Joystick joystick = new Joystick(0);
  private int loops = 0;
  private boolean lastBtn1 = false;
  private double targetPositionRotations;

  public giraffe() {
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                        kPIDLoopIdx, kTimeoutMs);
    talon.setSensorPhase(kSensorPhase);
    talon.setInverted(kMotorInvert);

    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configPeakOutputForward(1, kTimeoutMs);
    talon.configPeakOutputReverse(-1, kTimeoutMs);

    talon.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
  }

    void commonLoop() {
    //joystick and other stuff that should be in robotContainer
    double leftYStick = joystick.getY();
    boolean button1 = joystick.getRawButton(1); //X
    boolean button2 = joystick.getRawButton(2); //A

    //deadband
    if(Math.abs(leftYStick) < 0.05) {
      leftYStick = 0;
    }

  if (! lastBtn1 && button1) {
    targetPositionRotations = leftYStick * 10 * 2048;
    talon.set(ControlMode.Position, targetPositionRotations);
  }

  if (button2) {
    talon.set(TalonFXControlMode.PercentOutput, leftYStick);
  }

  if (++loops >=10) {
    loops = 0;
  }

  lastBtn1 = button1;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    commonLoop();
  }
}
