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
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.commands.DecreaseShooterSpeed;
import org.team2168.commands.IncreaseShooterSpeed;
import org.team2168.commands.SetTargetLocationSpeed;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {

  private DecreaseShooterSpeed Decrease = new DecreaseShooterSpeed(); 
  private IncreaseShooterSpeed Increase = new IncreaseShooterSpeed();
  private SetTargetLocationSpeed TargetLocation = new SetTargetLocationSpeed();

  private double commanded_speed_rpm = 0.0;

  @Log (rowIndex = 0, columnIndex = 0, width = 1, height = 1)
  private double actual_speed_rpm = 0.0;

  public WPI_TalonFX _motorRight;
  public WPI_TalonFX _motorLeft;

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
  TalonFXInvertType _motorOneInvert = TalonFXInvertType.CounterClockwise;
  TalonFXInvertType _motorTwoInvert = TalonFXInvertType.Clockwise;

  /** Config Objects for motor controllers */
  TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

  private static final double TICKS_PER_REV = 2048.0; //one event per edge on each quadrature channel
  private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  private static final double GEAR_RATIO = 18.0/24.0;  // motor pulley/shooter wheel pulley
  private static final double SECS_PER_MIN = 60.0;


  private double setPointVelocity_sensorUnits;

  /** Creates a new Shooter. */
  public Shooter() {

    _motorRight = new WPI_TalonFX(1);
    _motorLeft = new WPI_TalonFX(2);

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
    _motorRight.config_kF(kPIDLoopIdx, 0.52*1023.0/11205.0, kTimeoutMs);
    _motorRight.config_kP(kPIDLoopIdx, 1.0, kTimeoutMs);
    _motorRight.config_kI(kPIDLoopIdx, 0.0005, kTimeoutMs);
    _motorRight.config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
    _motorRight.config_IntegralZone(kPIDLoopIdx, 300, kTimeoutMs);

  }

    /**
     * Sets the closed loop shooter speed.
     * 
     * @param setPoint speed in RPM
     */
    public void setSpeed(double setPoint)
    {
        setPointVelocity_sensorUnits = revs_per_minute_to_ticks_per_100ms(setPoint);
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

  public double getVelocity()
  {
      return ticks_per_100ms_to_revs_per_minute(_motorRight.getSelectedSensorVelocity(kPIDLoopIdx));
  }

  public void setFiringLocation(double Speed, double[] set_Location) {
    set_Location = TargetLocation.locationCoords;
    Speed = TargetLocation.targetVelocity; 
  }

  public static getInstance(){
    if (_instance = null){
      _instance = new Shooter();
    }
    return _instance;
  }
}
