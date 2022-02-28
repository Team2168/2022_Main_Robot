// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants.CANDevices;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {

  public enum ShooterRPM {
    FENDER_LOW(1100.0),
    FENDER_HIGH(1500.0),
    TARMAC_LINE(1650.0),
    LAUNCHPAD(2085.0),
    WALL_SHOT(2750.0);

    public final double rpm;
    private ShooterRPM(double rpm) {
      this.rpm = rpm;
    }
  }

  public WPI_TalonFX _motorRight;
  public WPI_TalonFX _motorLeft;

  private double errorTolerance = 0.0;

  private StatorCurrentLimitConfiguration talonCurrentLimitStator;
  private final boolean ENABLE_CURRENT_LIMIT_STATOR = true;
  private final double CONTINUOUS_CURRENT_LIMIT_STATOR = 60; //amps
  private final double TRIGGER_THRESHOLD_LIMIT_STATOR = 70; //amp
  private final double TRIGGER_THRESHOLD_TIME_STATOR = 100; //ms

  private SupplyCurrentLimitConfiguration talonCurrentLimitSupply;
  private final boolean ENABLE_CURRENT_LIMIT_SUPPLY = true;
  private final double CONTINUOUS_CURRENT_LIMIT_SUPPLY = 45; //amps
  private final double TRIGGER_THRESHOLD_LIMIT_SUPPLY = 50; //amp
  private final double TRIGGER_THRESHOLD_TIME_SUPPLY = 0.2; //s

  private static Shooter _instance;

  public static final int kSlotIdx = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
   * now we just want the primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 30;

  public static final double max_velocity = 8000.0; //TODO set (measured ~18,000 units/1000ms at full stick)

  /** Invert Directions for Left and Right */
  TalonFXInvertType _motorOneInvert = TalonFXInvertType.Clockwise;
  TalonFXInvertType _motorTwoInvert = TalonFXInvertType.CounterClockwise;

  private static final double TICKS_PER_REV = 2048.0; //one event per edge on each quadrature channel
  private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  private static final double GEAR_RATIO = 24.0/18.0;  // motor pulley/shooter wheel pulley
  private static final double SECS_PER_MIN = 60.0;
  private static double velocityAdjustment = 0.0;


  /** Creates a new Shooter. */
  public Shooter() {

    _motorRight = new WPI_TalonFX(CANDevices.SHOOTER_RIGHT_MOTOR);
    _motorLeft = new WPI_TalonFX(CANDevices.SHOOTER_LEFT_MOTOR);

    /* Factory Default all hardware to prevent unexpected behaviour */
    _motorRight.configFactoryDefault();
    _motorLeft.configFactoryDefault();

    talonCurrentLimitStator = new StatorCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT_STATOR,
    CONTINUOUS_CURRENT_LIMIT_STATOR, TRIGGER_THRESHOLD_LIMIT_STATOR, TRIGGER_THRESHOLD_TIME_STATOR);
    
    _motorRight.configStatorCurrentLimit(talonCurrentLimitStator);
    _motorLeft.configStatorCurrentLimit(talonCurrentLimitStator);

    talonCurrentLimitSupply = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT_SUPPLY,
    CONTINUOUS_CURRENT_LIMIT_SUPPLY, TRIGGER_THRESHOLD_LIMIT_SUPPLY, TRIGGER_THRESHOLD_TIME_SUPPLY);
    _motorRight.configSupplyCurrentLimit(talonCurrentLimitSupply);
    _motorLeft.configSupplyCurrentLimit(talonCurrentLimitSupply);

    _motorRight.setNeutralMode(NeutralMode.Coast);
    _motorLeft.setNeutralMode(NeutralMode.Coast);

    /* Configure output and sensor direction */
    _motorRight.setInverted(_motorOneInvert);
    _motorLeft.setInverted(_motorTwoInvert);

    //set second motor as a follower
    _motorLeft.follow(_motorRight, FollowerType.PercentOutput);
    
    /* Config neutral deadband to be the smallest possible */
    _motorRight.configNeutralDeadband(0.01);

    /* Config sensor used for Primary PID [Velocity] */
    _motorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                        kPIDLoopIdx, 
                                        kTimeoutMs);
                                        

    /* Config the peak and nominal outputs */
    _motorRight.configNominalOutputForward(0.0, kTimeoutMs);
    _motorRight.configNominalOutputReverse(0.0, kTimeoutMs);
    _motorRight.configPeakOutputForward(1.0, kTimeoutMs);
    _motorRight.configPeakOutputReverse(0.0, kTimeoutMs); //set so that the shooter CANNOT run backwards

    /* Config the Velocity closed loop gains in slot0 */
    _motorRight.config_kF(kPIDLoopIdx, 0.41*1023.0/8570.0, kTimeoutMs);
    // feedforward; https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
    _motorRight.config_kP(kPIDLoopIdx, 0.25, kTimeoutMs);
    _motorRight.config_kI(kPIDLoopIdx, 0.0025, kTimeoutMs);
    _motorRight.config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
    _motorRight.config_IntegralZone(kPIDLoopIdx, 300, kTimeoutMs);
    // _motorRight.configMaxIntegralAccumulator(kPIDLoopIdx, iaccum, kTimeoutMs)

  }

  /**
   * Sets the closed loop shooter speed.
   * 
   * @param setPoint speed in RPM
   */
  public void setSpeed(double setPoint)
  {
      var setPointVelocity_sensorUnits = revs_per_minute_to_ticks_per_100ms(setPoint + velocityAdjustment);
      _motorRight.set(ControlMode.Velocity, setPointVelocity_sensorUnits);
  }

  
  public void setVelocityAdjustment(double adjustment) {
    velocityAdjustment = adjustment;
  }

  public void adjustVelocity(double delta) {
    setVelocityAdjustment(velocityAdjustment + delta);
  }
  /**
   * Bumps the shooter up by 50 RPM
   */
  public void incrementSpeed() {
    adjustVelocity(50.0);
  }

  /**
   * Bumps the shooter down by 50 RPM
   */
  public void decrementSpeed() {
    adjustVelocity(-50.0);
  }

  /**
   * Sets the bump amount of the shooter to zero
   */
  public void zeroSpeed() {
    setVelocityAdjustment(0.0);
  }


  /**
   * Convert speed in motor units per 100ms to RPM
   * 
   * @param ticks speed (ticks/100ms) to convert to RPM
   */
  private double ticks_per_100ms_to_revs_per_minute(double ticks) {
    //TODO: Verify conversion is correct
    return ticks * SECS_PER_MIN / (GEAR_RATIO * TICKS_PER_100MS);
  }

  /**
   * Converts RPM to sensor ticks per 100ms
   * 
   * @param revs speed (RPM) to convert to ticks/100ms
   */
  private double revs_per_minute_to_ticks_per_100ms(double revs) {
    return ((revs / 60.0 ) * 0.1) * GEAR_RATIO * TICKS_PER_REV;
    // return (revs / SECS_PER_MIN) * GEAR_RATIO * TICKS_PER_100MS;
  }

  /**
   * 
   * @return shooter speed in RPM
   */
  @Log (name = "Speed (RPM)", rowIndex = 0, columnIndex = 0, width = 1, height = 1)
  public double getVelocity() {
      return ticks_per_100ms_to_revs_per_minute(_motorRight.getSelectedSensorVelocity(kPIDLoopIdx));
  }

  /**
   * Sets the shooter at a speed
   * @param d_Speed the speed for the shooter to run at, from 0.0 to 1.0
   */
  public void shoot(double d_Speed){
    setSpeed(d_Speed);
    _motorRight.set(ControlMode.PercentOutput, Util.max(d_Speed, 0.0)); //prevent negative speeds from being commanded
  }

  public double getError() {
    return ticks_per_100ms_to_revs_per_minute(_motorRight.getClosedLoopError(kPIDLoopIdx));
  }

  /**
   * Checks if the shooter is at speed
   * If the shooter speed is 0, then it is not at speed
   * @param errorTolerance the allowed error for the shooter
   * @return whether the shooter is at speed
   */
  public boolean isAtSpeed(double errorTolerance) {
    this.errorTolerance = errorTolerance;
    //              within the allowed error           speed more than 0
    return ((Math.abs(getError()) < errorTolerance) && getVelocity() > 0);
  }


  /**
   * Checks if the shooter is at speed.
   * @return true when the shooter is within the errorTolerance specified by the last command
   */
  @Log(name = "At Speed?", columnIndex = 1, rowIndex = 1)
  public boolean isAtSpeed() {
    return Math.abs(getError()) < errorTolerance;
  }

  public static Shooter getInstance(){
    if (_instance == null){
      _instance = new Shooter();
    }
    return _instance;
  }
}
