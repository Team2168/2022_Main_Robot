// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import org.team2168.Constants;
import org.team2168.utils.CanDigitalInput;
import org.team2168.utils.Gains;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Turret extends SubsystemBase implements Loggable {
  /** Creates a new Turret. */
  private static CanDigitalInput hallEffectSensor;
  private static TalonFXHelper turretMotor;
  private static Turret instance = null;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 280.0/18.0 * 36.0/12.0; //(60.0/10.0) * (45.0/15.0); //TODO: update to match real gear ratio
  private static final double TICKS_PER_TURRET_ROTATION = TICKS_PER_REV * GEAR_RATIO;
  private static double setpoint = 0.0;

  private static final double TICKS_PER_SECOND = TICKS_PER_TURRET_ROTATION;
  private static final double TICKS_PER_100_MS = TICKS_PER_SECOND / 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60.0;
 
  private static final int MIN_ROTATION_TICKS = -73400;
  private static final int MAX_ROTATION_TICKS = 52200;

  private static final double ACCELERATION = degreesPerSecondToTicksPer100ms(360.0 * 3.0); //* 6;  
  private static final double CRUISE_VELOCITY = degreesPerSecondToTicksPer100ms(360.0 * 2.0);

  //gains
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = true;
  public static boolean kMotorInvert = false;

  //                                     P,   I,   D,   F,  I zone, and Peak output
  private static final Gains kGains;
  static {
    if (Constants.IS_COMPBOT) {
      kGains = new Gains(0.5, 0.0, 0.0, 0.0, 0, 1.0);
    } else {
      kGains = new Gains(0.5, 0.0, 0.0, 0.0, 0, 1.0);
    }
  }

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amps
  private final double TRIGGER_THRESHOLD_TIME = 0.02; //seconds

  //Simulation objects
  // Characterization
  public static final double KV = 0.05;
  public static final double KA= 0.002;

  private static FlywheelSim turretSim;
  private static TalonFXSimCollection turretMotorSim;

  private Turret() {
    turretMotor = new TalonFXHelper(Constants.CANDevices.TURRET_MOTOR);
    hallEffectSensor = new CanDigitalInput(turretMotor);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    //Configuring the turret motor
    turretMotor.configFactoryDefault();

    turretMotor.configClosedLoopStatusFrameRates();

    turretMotor.configSupplyCurrentLimit(talonCurrentLimit);

    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    turretMotor.setSensorPhase(kSensorPhase);
    turretMotor.setInverted(kMotorInvert);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configNeutralDeadband(0.001);

    turretMotor.configForwardSoftLimitThreshold(MAX_ROTATION_TICKS);
    turretMotor.configReverseSoftLimitThreshold(MIN_ROTATION_TICKS);
    turretMotor.configForwardSoftLimitEnable(true, 0);
    turretMotor.configReverseSoftLimitEnable(true, 0);

    turretMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    turretMotor.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    turretMotor.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    turretMotor.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    turretMotor.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);

    turretMotor.configMotionAcceleration(ACCELERATION);
    turretMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);

    //Setup simulation
    turretSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(KV, KA),
      DCMotor.getFalcon500(1),
      GEAR_RATIO
    );
    turretMotorSim = turretMotor.getSimCollection();
    
  }

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  @Log (name = "At Zero", rowIndex = 3, columnIndex = 0)
  public boolean isTurretAtZero() {
    return (hallEffectSensor.isFwdLimitSwitchClosed() && getEncoderPosition() < 10 && getEncoderPosition() > -10);
  }

  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Rotates the turret to a position
   * 
   * @param rotation Between -1 and 1, -1 is a -360 degree rotation, 0 is no movement, and 1 is a 360 degree rotation
   */

  // TODO: Verify - I don't think this is right, the motor's encoder is a continuous measurement, this command is creating relative movement.
  //  e.g. 0 = no motion. for this to work you'd need to be commanding an offset to the current encoder position in the motor.  
  // public void setRotation(double rotation) {
  //   turretMotor.set(ControlMode.MotionMagic, (rotation * TICKS_PER_WHEEL_ROTATION) /10);
  // }

  /**
   * Command the turret to an absolute position relative to the zero position sensor
   * @param degrees the destination position (degrees)
   */
  public void setRotationDegrees(double degrees) {
    //CHECK THIS
    var demand = MathUtil.clamp(degrees, ticksToDegrees(-MAX_ROTATION_TICKS), ticksToDegrees(MAX_ROTATION_TICKS));
    setpoint = degrees;
    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(demand));
  }

  /**
   * convert a position in degrees to units the motor understands
   * @param degrees the absolute position (degrees)
   * @return the position in ticks (F500 internal encoder)
   */
  private static double degreesToEncoderTicks(double degrees) {
    return (degrees / 360.0) * TICKS_PER_TURRET_ROTATION;
  }

  /**
   * Convert motor ticks to a mechanism position in degrees
   * @param ticks the absolute position in f500 internal encoder ticks
   * @return the absolute position (degrees)
   */
  private static double ticksToDegrees(double ticks) {
    return (ticks / TICKS_PER_TURRET_ROTATION) * 360.0;
  }

  /**
   * Convert a motor velocity to a human readable one
   * @param ticks in native motor controller units (ticks/100mS)
   * @return velocity in degrees/second
   */
  private static double ticksPer100msToDegreesPerSec(double ticks) {
    return ticksToDegrees(ticks) * 10.0;
  }
  
  /**
   * Converts a velocity to native sensor units.
   * @param degrees the target velocity (degrees/s)
   * @return the desired speed in ticks/100ms 
   */
  public static double degreesPerSecondToTicksPer100ms(double degrees) {
    return degreesToEncoderTicks(degrees) / 10.0;
  }

  /**
   * Sets the velocity
   * @param degrees degrees per second the motor should run
   */
  public void setVelocity(double degrees) {
    turretMotor.set(ControlMode.Velocity, degreesPerSecondToTicksPer100ms(degrees));
  }

  public void drive(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  @Log(name = "Error (deg)", rowIndex = 4, columnIndex = 2)
  /**
   * @return the current position error in degrees
   */
  public double getControllerError() {
    return ticksToDegrees(turretMotor.getClosedLoopError());
  }

  /**
   * 
   * @return the turret position in degrees relative to the zero position sensor
   */
  @Log(name = "Position (deg)", rowIndex = 3, columnIndex = 2)
  public double getPositionDegrees() {
    return ticksToDegrees(turretMotor.getSelectedSensorPosition());
  }

  /**
   * 
   * @return the turret velocity in degrees per second
   */
  @Log(name = "Speed (deg-s)", rowIndex = 3, columnIndex = 3)
  public double getVelocityDegPerSec() {
    return ticksPer100msToDegreesPerSec(turretMotor.getSelectedSensorVelocity());
  }

   /**
   * 
   * @return The internal sensor's position
   */
  @Log(name = "Encoder Position", rowIndex = 3, columnIndex = 4)
  public double getEncoderPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  public void zeroEncoder() {
    turretMotor.setSelectedSensorPosition(0.0);
  }

  public boolean atSoftLimit() {
    return !(getEncoderPosition() < MAX_ROTATION_TICKS && getEncoderPosition() > -MAX_ROTATION_TICKS);
  }

  /**
   * Calculates how much the turret should turn if the quickest path conflicts with the turret's soft limits
   * @param targetPos the distance away from the target in degrees
   * @return the amount from a zero'd turret to rotate
   */
  public double amountFromZeroToRotate(double targetPos) {
    if (targetPos > 0) 
      return targetPos - 360;
    else
      return targetPos + 360; 
  }

  public void zeroTurret() {
    while (!isTurretAtZero())
      setRotationDegrees(-getEncoderPosition());
  }

  public double getForwardSoftLimit() {
    return ticksToDegrees(MAX_ROTATION_TICKS);
  }

  public double getReverseSoftLimit() {
    return ticksToDegrees(MIN_ROTATION_TICKS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Only zeros the turret if it is actually at zero and not at 360/-360
    if (isTurretAtZero())
      zeroEncoder();
  }

  @Override
  public void simulationPeriodic() {
    // Affect motor outputs by main system battery voltage dip 
    turretMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // Pass motor output voltage to physics sim
    turretSim.setInput(turretMotorSim.getMotorOutputLeadVoltage());
    turretSim.update(Constants.LOOP_TIMESTEP_S);

    // Update motor sensor states based on physics model
    double sim_velocity_ticks_per_100ms = turretSim.getAngularVelocityRPM() * ONE_HUNDRED_MS_PER_MINUTE;
    turretMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100ms);
    turretMotorSim.setIntegratedSensorRawPosition((int) (getEncoderPosition() +
      Constants.LOOP_TIMESTEP_S * sim_velocity_ticks_per_100ms));

  }
}
