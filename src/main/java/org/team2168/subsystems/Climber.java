// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.team2168.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber extends SubsystemBase implements Loggable {
  static Climber instance = null;

  /** Creates a new Climber. */
  private static WPI_TalonFX climbMotor1 = new WPI_TalonFX(Constants.CANDevices.CLIMBER_MOTOR_1); //left motor when looking at the output shafts
  private static WPI_TalonFX climbMotor2 = new WPI_TalonFX(Constants.CANDevices.CLIMBER_MOTOR_2); //right motor when looking at the output shafts

  private static DigitalInput climbHooks = new DigitalInput(Constants.DIO.CLIMBER_HOOK_LIMIT_SWITCH);

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = (40.0 / 10.0) * (40.0 / 14.0) * (24.0 / 24.0);
  private static final double SPROCKET_RADIUS_INCHES = 0.6589;
  private static final double INCHES_PER_REV = SPROCKET_RADIUS_INCHES * 2 * Math.PI;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static boolean kSensorPhase = false;
  private static TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise; // direction of output shaft rotation when looking at
                                                                                      // the motor face w/ shaft that causes the lift to go up

  private static final double TIME_UNITS_OF_VELOCITY = 0.1; // in seconds

  // Gains
  private static final double kP = 0.2;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;
  private static final double kArbitraryFeedForward = 0.034; // 0.034
  private static final int kIzone = 0;
  private static final double kPeakOutput = 1.0;
  private static final double NEUTRAL_DEADBAND = 0.01;
  private static final double ACCELERATION_LIMIT = inchesToTicks(6.0) * TIME_UNITS_OF_VELOCITY;     // TODO: Change when mechanism is avaialble
  private static final double CRUISE_VELOCITY_LIMIT = inchesToTicks(12.0) * TIME_UNITS_OF_VELOCITY; // TODO: Change when mechanism is avaialble
  // private static final int S_CURVE_STRENGTH = 0; // determines the shape of the motion magic graph

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
  private static final double MIN_HEIGHT_INCHES = 0.0;
  private static final double MAX_HEIGHT_INCHES = 40.0;
  
  private Climber() {
    climbMotor1.configFactoryDefault();
    climbMotor2.configFactoryDefault();
    climbMotor1.configNeutralDeadband(NEUTRAL_DEADBAND);
    climbMotor2.configNeutralDeadband(NEUTRAL_DEADBAND);
    
    climbMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    climbMotor1.setSensorPhase(kSensorPhase);
    climbMotor1.setInverted(kMotorInvert);

    climbMotor1.configNominalOutputForward(0, kTimeoutMs);
    climbMotor1.configNominalOutputReverse(0, kTimeoutMs);
    climbMotor1.configPeakOutputForward(kPeakOutput, kTimeoutMs);
    climbMotor1.configPeakOutputReverse(-kPeakOutput, kTimeoutMs);
    
    climbMotor1.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    climbMotor1.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    climbMotor1.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    climbMotor1.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    climbMotor1.configMotionAcceleration(ACCELERATION_LIMIT);
    climbMotor1.configMotionCruiseVelocity(CRUISE_VELOCITY_LIMIT);
    // climbMotor1.configMotionSCurveStrength(S_CURVE_STRENGTH);
    climbMotor1.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    climbMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climbMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
        CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    climbMotor1.configSupplyCurrentLimit(talonCurrentLimit);
    climbMotor2.configSupplyCurrentLimit(talonCurrentLimit);

    // Tells second climber motor to do the same outputs as the first climber motor,
    // and at the same time.
    climbMotor2.set(ControlMode.Follower, Constants.CANDevices.CLIMBER_MOTOR_1);
    climbMotor2.setInverted(InvertType.OpposeMaster);

    m_climberSim = new ElevatorSim(
        DCMotor.getFalcon500(2),
        GEAR_RATIO,
        CARRIAGE_MASS_KG,
        Units.inchesToMeters(SPROCKET_RADIUS_INCHES),
        Units.inchesToMeters(MIN_HEIGHT_INCHES),
        Units.inchesToMeters(MAX_HEIGHT_INCHES)
    );

    m_climberMotorSim = climbMotor1.getSimCollection();
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  /**
   * 
   * @return true when the lift is fully lowered
   */
  @Log (name = "At Zero", rowIndex = 3, columnIndex = 0)
  public boolean isAtZeroPosition() {
    return climbMotor1.isRevLimitSwitchClosed() == 1;
  }

  public boolean isAtUpperPosition() {
    return climbMotor1.isFwdLimitSwitchClosed() == 1;
  }

  // methods that convert a velocity value from seconds to hundreds of
  // milliseconds and back
  private static double convertVelocitySecondstoHundredMs(double speed) {
    return speed * TIME_UNITS_OF_VELOCITY;
  }

  private static double convertVelocityHundredMstoSeconds(double speed) {
    return speed / TIME_UNITS_OF_VELOCITY;
  }

  /**
   * 
   * @return the leader motor position in native ticks
   */
  private double getEncoderTicksMotor1() {
    return climbMotor1.getSelectedSensorPosition(kPIDLoopIdx);
  }

  /**
   * 
   * @return the follower position in native ticks
   */
  private double getEncoderTicksMotor2() {
    return climbMotor2.getSelectedSensorPosition(kPIDLoopIdx);
  }

  /**
   * Sets the internal sensors of Falcon 500 back to 0.
   * Used when the lift is at a zero position.
   */
  public void setEncoderPosZero() {
    climbMotor1.setSelectedSensorPosition(0.0);
    climbMotor2.setSelectedSensorPosition(0.0);
  }

  /**
   * 
   * @param inches the lift position in inches
   * @return position in F500 internal encoder ticks
   */
  private static double inchesToTicks(double inches) {
    return (inches / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  /**
   * 
   * @param ticks the lift position in F500 interal encoder ticks
   * @return the lift position in inches
   */
  private static double ticksToInches(double ticks) {
    return (ticks / TICKS_PER_REV) / GEAR_RATIO * INCHES_PER_REV;
  }

  /**
   * 
   * @return current lift velocity (inches/second)
   */
  @Log(name = "Speed (In-s)", rowIndex = 3, columnIndex = 3)
  public double getSpeedInchesPerSecond() {
    return convertVelocityHundredMstoSeconds(ticksToInches(climbMotor1.getSelectedSensorVelocity()));
  }

  /**
   * 
   * @return current lift position (inches), zero is fully lowered
   */
  @Log(name = "Position (In)", rowIndex = 3, columnIndex = 2)
  public double getPositionInches() {
    return ticksToInches(climbMotor1.getSelectedSensorPosition());
  }

  // 
  /**
   * Commands the lift at a specified velocity.
   * @param speedInInchesPerSec speed to run the lift at, positive up.
   */
  public void setSpeed(double speedInInchesPerSec) {
    climbMotor1.set(ControlMode.Velocity, inchesToTicks(speedInInchesPerSec) * TIME_UNITS_OF_VELOCITY);
  }

  /**
   * Commands the lift to a specied position relative to the zero position.
   * @param inches the position to move the lift to, positive up.
   */
  public void setPosition(double inches) {
    climbMotor1.set(ControlMode.MotionMagic, inchesToTicks(inches),
        DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  /**
   * Command the motor in open loop mode. 
   * @param speed percentage of bus voltage to output 1.0 to -1.0
   */
  public void setPercentOutput(double speed) {
    climbMotor1.set(ControlMode.PercentOutput, speed);
  }

  public double getArbitraryFeedforward() {
    return kArbitraryFeedForward;
  }

  /**
   * 
   * @return the value of the collective limit switches on the climber Hooks
   */
  @Log(name = "Is Climber Attached?", rowIndex = 3, columnIndex = 4)
  public boolean isClimberHookAttached() {
    return !climbHooks.get();
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

    // System.out.println("Climber pos: " + m_climberSim.getPositionMeters());

    // Update motor sensor states based on physics model
    double sim_velocity_ticks_per_100_ms = inchesToTicks(Units.metersToInches(m_climberSim.getVelocityMetersPerSecond())) * TIME_UNITS_OF_VELOCITY;
    double sim_position = inchesToTicks(Units.metersToInches(m_climberSim.getPositionMeters()));
    m_climberMotorSim.setIntegratedSensorRawPosition((int) sim_position);
    m_climberMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100_ms);

    // Set simulated limit switch positions from simulation methods
    // m_climberMotorSim.setLimitRev(m_climberSim.hasHitLowerLimit());
    // m_climberMotorSim.setLimitFwd(m_climberSim.hasHitUpperLimit());
    // m_climberMotorSim.setLimitRev(sim_position <= MIN_HEIGHT_INCHES + 0.1);
    // m_climberMotorSim.setLimitFwd(sim_position >= MAX_HEIGHT_INCHES - 0.1);
  }
}
