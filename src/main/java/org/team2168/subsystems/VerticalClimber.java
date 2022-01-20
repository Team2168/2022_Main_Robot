// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import org.team2168.RobotContainer;
import org.team2168.Constants;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VerticalClimber extends SubsystemBase {
  static VerticalClimber instance = null;

  /** Creates a new Giraffe. */
  public WPI_TalonFX climbMotor1 = new WPI_TalonFX(Constants.CANDevices.VERTICAL_CLIMB_MOTOR_1);
  public WPI_TalonFX climbMotor2 = new WPI_TalonFX(Constants.CANDevices.VERTICAL_CLIMB_MOTOR_2);

  /** Track button state for single press event */
	boolean _lastButton1 = false;

  /** Save the target position to servo to */
	double targetPositionRotations;

  public final int TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 1.0;
  private final double TICKS_PER_WHEEL_ROTATION = TICKS_PER_REV * GEAR_RATIO;

  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = true;
  public static boolean kMotorInvert = false;

  public final double TIME_UNITS_OF_VELOCITY = 0.1; // in seconds
  
  // Gains
  public final double kP = 0.15;
  public final double kI = 0.0;
  public final double kD = 1.0;
  public final double kF = 0.0;
  public final int kIzone = 0;
  public final double kPeakOutput = 1.0;

  public void setPositionControlMode() {
    climbMotor1.set(ControlMode.MotionMagic, 0);
  }

  public int CheckLimitSwitchClosed() {
     return climbMotor1.isFwdLimitSwitchClosed();
  }

  public double convertVelocitySecondstoHundredMs(double speed) {
    return speed * TIME_UNITS_OF_VELOCITY;
  }

  public double convertHundredMstoVelocitySeconds(double speed) {
    return speed / TIME_UNITS_OF_VELOCITY;
  }

  public VerticalClimber() {
    climbMotor1.configFactoryDefault();
    climbMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    climbMotor1.setSensorPhase(kSensorPhase);
    climbMotor1.setInverted(kMotorInvert);

    climbMotor1.configNominalOutputForward(0, kTimeoutMs);
		climbMotor1.configNominalOutputReverse(0, kTimeoutMs);
		climbMotor1.configPeakOutputForward(1, kTimeoutMs);
		climbMotor1.configPeakOutputReverse(-1, kTimeoutMs);

    climbMotor1.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    climbMotor1.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		climbMotor1.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		climbMotor1.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		climbMotor1.config_kD(kPIDLoopIdx, kD, kTimeoutMs);


    climbMotor2.configFactoryDefault();
    climbMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    climbMotor2.setSensorPhase(kSensorPhase);
    climbMotor2.setInverted(kMotorInvert);

    climbMotor2.configNominalOutputForward(0, kTimeoutMs);
		climbMotor2.configNominalOutputReverse(0, kTimeoutMs);
		climbMotor2.configPeakOutputForward(1, kTimeoutMs);
		climbMotor2.configPeakOutputReverse(-1, kTimeoutMs);

    climbMotor2.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    climbMotor2.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		climbMotor2.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		climbMotor2.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		climbMotor2.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

    climbMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climbMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    climbMotor2.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climbMotor2.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);


  }

  public static VerticalClimber getInstance() {
    if (instance == null) {
      instance = new VerticalClimber();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
