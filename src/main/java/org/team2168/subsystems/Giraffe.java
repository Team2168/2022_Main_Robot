// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import org.team2168.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Giraffe extends SubsystemBase {
  /** Creates a new Giraffe. */
  public WPI_TalonFX giraffeMotor = new WPI_TalonFX(0);

  /** Track button state for single press event */
	boolean _lastButton1 = false;

  /** Save the target position to servo to */
	double targetPositionRotations;

  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = true;
  public static boolean kMotorInvert = false;
  
  // Gains
  public final double kP = 0.15;
  public final double kI = 0.0;
  public final double kD = 1.0;
  public final double kF = 0.0;
  public final int kIzone = 0;
  public final double kPeakOutput = 1.0;

  public Giraffe() {
    giraffeMotor.configFactoryDefault();
    giraffeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    giraffeMotor.setSensorPhase(kSensorPhase);
    giraffeMotor.setInverted(kMotorInvert);

    giraffeMotor.configNominalOutputForward(0, kTimeoutMs);
		giraffeMotor.configNominalOutputReverse(0, kTimeoutMs);
		giraffeMotor.configPeakOutputForward(1, kTimeoutMs);
		giraffeMotor.configPeakOutputReverse(-1, kTimeoutMs);

    giraffeMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    giraffeMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		giraffeMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		giraffeMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		giraffeMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
  }

  void commonLoop() {
		/* Gamepad processing */
		double leftYstick = RobotContainer.driverJoystick.getY();
		boolean button1 = RobotContainer.driverJoystick.getRawButton(1);	// X-Button
		boolean button2 = RobotContainer.driverJoystick.getRawButton(2);	// A-Button

		/* Get Talon's current output percentage */
		double motorOutput = giraffeMotor.getMotorOutputPercent();
		/* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}

    if (!_lastButton1 && button1) {
			/* Position Closed Loop */

			/* 10 Rotations * 2048 u/rev in either direction */
			targetPositionRotations = leftYstick * 10.0 * 2048;
			giraffeMotor.set(TalonFXControlMode.Position, targetPositionRotations);
		}

    if (button2) {
			/* Percent Output */

			giraffeMotor.set(TalonFXControlMode.PercentOutput, leftYstick);
		}

    /* Save button state for on press detect */
		_lastButton1 = button1;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    commonLoop();
  }
}
