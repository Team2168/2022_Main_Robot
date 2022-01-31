// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants;
import org.team2168.utils.CanDigitalInput;
import org.team2168.utils.Gains;

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
  private static WPI_TalonFX turretMotor;
  private static Turret instance = null;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 1.0;
  private static final double TICKS_PER_TURRET_ROTATION = TICKS_PER_REV * GEAR_RATIO;

  private static final double TICKS_PER_SECOND = TICKS_PER_TURRET_ROTATION;
  private static final double TICKS_PER_100_MS = TICKS_PER_SECOND / 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60.0;
 
  //About 260/360 degrees
  private static final int MAX_ROTATION_TICKS = 1480;

  private static final double ACCELERATION = TICKS_PER_100_MS;  // TODO: Change when mechanism is avaialble
  private static final double CRUISE_VELOCITY = TICKS_PER_100_MS; // TODO: Change when mechanism is avaialble

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

  //Simulation objects
  // Characterization
  public static final double KV = 0.05;
  public static final double KA= 0.002;

  private static FlywheelSim m_turretSim;
  private static TalonFXSimCollection m_turretMotorSim;

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
    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    turretMotor.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    turretMotor.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    turretMotor.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    turretMotor.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);

    turretMotor.configMotionAcceleration(ACCELERATION);
    turretMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);

    //Setup simulation
    m_turretSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(KV, KA),
      DCMotor.getFalcon500(1),
      GEAR_RATIO
    );
    m_turretMotorSim = turretMotor.getSimCollection();
    
  }

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  @Log (name = "At Zero", rowIndex = 3, columnIndex = 0)
  public boolean isTurretAtZero() {
    return hallEffectSensor.isFwdLimitSwitchClosed();
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
    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(degrees));
  }

  /**
   * convert a position in degrees to units the motor understands
   * @param degrees 
   * @return 
   */
  private double degreesToEncoderTicks(double degrees) {
    return (degrees / 360.0) * TICKS_PER_TURRET_ROTATION;
  }

  /**
   * Convert motor ticks to a mechanism position in degrees
   * @param ticks 
   * @return
   */
  private double ticksToDegrees(double ticks) {
    return (ticks / TICKS_PER_TURRET_ROTATION) * 360.0;
  }

  /**
   * Convert a motor velocity to a human readable one
   * @param ticks in native motor controller units (ticks/100mS)
   * @return velocity in degrees/second
   */
  private double ticksPer100msToDegreesPerSec(double ticks) {
    return ticksToDegrees(ticks) * 10.0;
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

  public void zeroEncoder() {
    turretMotor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Affect motor outputs by main system battery voltage dip 
    m_turretMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // Pass motor output voltage to physics sim
    m_turretSim.setInput(m_turretMotorSim.getMotorOutputLeadVoltage());
    m_turretSim.update(Constants.LOOP_TIMESTEP_S);

    // Update motor sensor states based on physics model
    double sim_velocity_ticks_per_100ms = m_turretSim.getAngularVelocityRPM() * ONE_HUNDRED_MS_PER_MINUTE;
    m_turretMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100ms);
    m_turretMotorSim.setIntegratedSensorRawPosition((int) (getEncoderPosition() + 
      Constants.LOOP_TIMESTEP_S * sim_velocity_ticks_per_100ms));

  }
}