// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.team2168.Constants;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Climber extends SubsystemBase implements Loggable {
  static Climber instance = null;

  /** Creates a new Climber. */
  private static WPI_TalonFX climbMotor1 = new WPI_TalonFX(Constants.CANDevices.CLIMBER_MOTOR_1);
  private static WPI_TalonFX climbMotor2 = new WPI_TalonFX(Constants.CANDevices.CLIMBER_MOTOR_2);

  private static final int TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = (10.0 / 40.0) * (14.0 / 40.0) * (18.0 / 24.0);
  private static final double SPROCKET_RADIUS_INCHES = 0.6589;
  private static final double INCHES_PER_REV = SPROCKET_RADIUS_INCHES * 2 * Math.PI;
  private static final double TICKS_PER_WHEEL_ROTATION = TICKS_PER_REV * GEAR_RATIO;
  private static final double MIN_HEIGHT_INCHES = 0.0;
  private static final double MAX_HEIGHT_INCHES = 40.0;

  private static final int kSlotIdx = 0;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static boolean kSensorPhase = true;
  private static boolean kMotorInvert = false;

  private static final double TIME_UNITS_OF_VELOCITY = 0.1; // in seconds
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60.0;

  // Gains
  private static final double kP = 0.15;
  private static final double kI = 0.0;
  private static final double kD = 1.0;
  private static final double kF = 0.07;
  private static final int kIzone = 0;
  private static final double kPeakOutput = 1.0;

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; // amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; // amp
  private final double TRIGGER_THRESHOLD_TIME = 0.2; // s

  //Simulation objects
  // Characterization
  private static ElevatorSim m_climberSim;
  private static TalonFXSimCollection m_climberMotorSim;
  private static final double CARRIAGE_MASS_KG = 4.5;
  
  private Climber() {
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

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
        CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    climbMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    climbMotor2.configSupplyCurrentLimit(talonCurrentLimit);

    // Tells second climber motor to do the same outputs as the first climber motor,
    // and at the same time.
    climbMotor2.set(ControlMode.Follower, Constants.CANDevices.CLIMBER_MOTOR_1);

    m_climberSim = new ElevatorSim(
        DCMotor.getFalcon500(2),
        GEAR_RATIO,
        CARRIAGE_MASS_KG,
        Units.inchesToMeters(SPROCKET_RADIUS_INCHES),
        Units.inchesToMeters(MIN_HEIGHT_INCHES),
        Units.inchesToMeters(MAX_HEIGHT_INCHES),
        VecBuilder.fill(0.01)
    );

    m_climberMotorSim = climbMotor1.getSimCollection();
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  // checks if the climber is at the zero position
  public boolean isAtZeroPosition() {
    if (climbMotor1.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }

  // methods that convert a velocity value from seconds to hundreds of
  // milliseconds and back
  public double convertVelocitySecondstoHundredMs(double speed) {
    return speed * TIME_UNITS_OF_VELOCITY;
  }

  public double convertVelocityHundredMstoSeconds(double speed) {
    return speed / TIME_UNITS_OF_VELOCITY;
  }

  // returns encoder ticks for each of the motors
  public double getEncoderTicksMotor1() {
    return climbMotor1.getSelectedSensorPosition(kPIDLoopIdx);
  }

  public double getEncoderTicksMotor2() {
    return climbMotor2.getSelectedSensorPosition(kPIDLoopIdx);
  }

  // sets internal sensors of Falcon 500 back to 0 - used when the lift is at a
  // zero position.
  public void setEncoderPosZero() {
    climbMotor1.setSelectedSensorPosition(0.0);
    climbMotor2.setSelectedSensorPosition(0.0);
  }

  // commands the lift to a certain velocity (in inches per second)
  public void setSpeed(double speedInInchesPerSec) {
    climbMotor1.set(ControlMode.Velocity,
        (speedInInchesPerSec / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV * TIME_UNITS_OF_VELOCITY,
        DemandType.ArbitraryFeedForward, kF);
  }

  // commands the lift to a certain position (in inches from the zero position)
  public void setPosition(double inches) {
    climbMotor1.set(ControlMode.MotionMagic, (inches / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV,
        DemandType.ArbitraryFeedForward, kF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Affect motor outputs by main system battery voltage dip 
    m_climberMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // Pass motor output voltage to physics sim
    m_climberSim.setInput(m_climberMotorSim.getMotorOutputLeadVoltage());
    m_climberSim.update(Constants.LOOP_TIMESTEP_S);

    // Update motor sensor states based on physics model
    // TODO: update the motor sim's position and velocity values using the elevatorsim's position and velocity.
    //       in other words convert elevatorSim's units (meters) to TalonFX native units (ticks)
    // m_climberSim.getPositionMeters()
    // m_climberSim.getVelocityMetersPerSecond()

    //How it was done for the turret:
    // double sim_velocity_ticks_per_100ms = m_climberSim.getAngularVelocityRPM() * ONE_HUNDRED_MS_PER_MINUTE;
    // m_climberMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100ms);
    // m_climberMotorSim.setIntegratedSensorRawPosition((int) (getEncoderTicksMotor1() + 
    //   Constants.LOOP_TIMESTEP_S * sim_velocity_ticks_per_100ms));

    //TODO: set simulated limit switch positions from simulation methods
    // m_climberSim.hasHitLowerLimit()
    // m_climberSim.hasHitUpperLimit()
  }
}
