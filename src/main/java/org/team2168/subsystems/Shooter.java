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

import org.team2168.Constants;
import org.team2168.Constants.CANDevices;
import org.team2168.utils.TalonFXHelper;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {

  public enum ShooterRPM {
    AUTO_LOADING_ZONE(2450.0,false),
    AUTO_TARMAC_LINE(1745.0, false), // 2.65 meters from hub
    AUTO_BALL3(1893, false),         // ?? meters from hub
    AUTO_BALL4(2000.5, false),       // 3.45 meters from hub
    AUTO_LAUNCHPAD(2335.0, true),
    AUTO_4_BALL(1850, false),
    FENDER_LOW(800.0, false),//(1050),//(900.0),
    FENDER_HIGH(1625.0, true),//(1500.0),
    TARMAC_LINE(1770.0, false),  // 1650
    LAUNCHPAD(2000.0, true),//(2085.0),
    WALL_SHOT(2750.0, true),
    TERMINAL(2300.0, true),
    // ONE_METER_FROM_HUB(1625.0, true),
    // TWO_METER_FROM_HUB(1775.0, true), // PBOT 1840.0, true
    // THREE_METER_FROM_HUB(1950.0, true), // PBOT 1920.0, true
    // FOUR_METER_FROM_HUB(2075.0, true), // PBOT 2080.0, true
    // FIVE_METER_FROM_HUB(2575.0, true), // PBOT 2575.0, true
    STOP(0.0, true);
    // FENDER_LOW_CBOT(1100.0),  // TODO fix this once we have pbot jumper merged
    // FENDER_HIGH_CBOT(1500.0),
    // TARMAC_LINE_CBOT(1550),//PBot (1650.0),
    // LAUNCHPAD_CBOT (1870),//PBot(2085.0),
    // WALL_SHOT_CBOT(2500),//PBot(2750.0);
    // TERMINAL_CBOT(2300.0);

    public final double rpm;
    public final boolean waitForShpooterAtSpeed;
    private ShooterRPM(double rpm, boolean waitForShooterAtSpeed) {
      this.rpm = rpm;
      this.waitForShpooterAtSpeed = waitForShooterAtSpeed;
    }
  }

  public TalonFXHelper _motorRight;
  public TalonFXHelper _motorLeft;

  private double errorTolerance = 5.0; // Default shooter speed error tolerance +/- target (RPM)

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

  public static final double kF;
  public static final double kP;
  public static final double kI;
  public static final double kD;
  public static final double INTEGRAL_ZONE;
  static {
    if (Constants.IS_COMPBOT) {
      kF = 0.41*1023.0/7512;
      kP = 0.25;
      kI = 0.0025;
      kD = 0.0;
      INTEGRAL_ZONE = 150.0;
    } else {
      kF = 0.41*1023.0/8570.0;
      kP = .11;//0.25;
      kI = 0.001;//0.0025;
      kD = 0.0;
      INTEGRAL_ZONE = 600;//300.0;
    }
  }
  // private double setPoint_RPM;
  private double setpoint = 0.0;
  private boolean waitForShpooterAtSpeed = true;

  /** Creates a new Shooter. */
  public Shooter() {

    _motorRight = new TalonFXHelper(CANDevices.SHOOTER_RIGHT_MOTOR);
    _motorLeft = new TalonFXHelper(CANDevices.SHOOTER_LEFT_MOTOR);

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
    _motorRight.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    _motorRight.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    _motorRight.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    _motorRight.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    _motorRight.config_IntegralZone(kPIDLoopIdx, INTEGRAL_ZONE, kTimeoutMs);

    // Reduce can status frame rates
    _motorLeft.configFollowerStatusFrameRates();
    _motorRight.configClosedLoopStatusFrameRates();
  }

  /**
   * Sets the closed loop shooter speed.
   * 
   * @param setPoint speed preset
   */
  public void setSpeed(ShooterRPM setPoint) {
    this.setpoint = setPoint.rpm;
    this.waitForShpooterAtSpeed = setPoint.waitForShpooterAtSpeed;
    var setPointVelocity_sensorUnits = revs_per_minute_to_ticks_per_100ms(setpoint);
    _motorRight.set(ControlMode.Velocity, setPointVelocity_sensorUnits);
  }

  public void setSpeed(double speed) {
    this.setpoint = speed;
    var setPointVelocity_sensorUnits = revs_per_minute_to_ticks_per_100ms(setpoint);
    _motorRight.set(ControlMode.Velocity, setPointVelocity_sensorUnits);
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
   * 
   * @return target speed in RPM
   */
  public double getSetPoint() {
    return setpoint;
  }

  @Log(name = "Error (RPM)", columnIndex = 2, rowIndex = 1)
  public double getError() {
    return ticks_per_100ms_to_revs_per_minute(_motorRight.getClosedLoopError(kPIDLoopIdx));
  }

  /**
   * Checks if the shooter is at speed
   * @param errorTolerance the allowed error for the shooter
   * @return whether the shooter is at speed
   */
  public boolean isAtSpeed(double errorTolerance) {
    this.errorTolerance = errorTolerance;
    return (Math.abs(getError()) < errorTolerance) && (getSetPoint() != 0.0);
  }

  public boolean shouldWaitForShooterAtSpeed() {
    return waitForShpooterAtSpeed;
  }

  public void setWaitForShooterAtSpeed(boolean wait) {
    waitForShpooterAtSpeed = wait;
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
