// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final boolean _INDEXER_MOTOR_REVERSED = false;
  private WPI_TalonFX indexerMotor;

  @Log
  private static DigitalInput entranceLineBreak;
  @Log
  private static DigitalInput exitLineBreak;

  private static Indexer _instance = null;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amp
  private final double TRIGGER_THRESHOLD_TIME = 0.02; //s

  public Indexer() {
    entranceLineBreak = new DigitalInput(Constants.Sensors.ENTRANCE_LINE_BREAK);
    exitLineBreak = new DigitalInput(Constants.Sensors.EXIT_LINE_BREAK);

    indexerMotor = new WPI_TalonFX(Constants.CANDevices.INDEXER_MOTOR_PDP);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    indexerMotor.configFactoryDefault();
    indexerMotor.configSupplyCurrentLimit(talonCurrentLimit);

    indexerMotor.setNeutralMode(NeutralMode.Brake);
    indexerMotor.setInverted(true); //output polarity is apparently reversed between SRX and SPARK MAX?
    indexerMotor.configNeutralDeadband(0.05);

  }

  public static Indexer getInstance(){
    if(_instance == null){
      _instance = new Indexer();
    }
    return _instance;
  }  

  public void drive(double speed) {
      if(_INDEXER_MOTOR_REVERSED) {
        speed = speed * -1;
      }
      indexerMotor.set(ControlMode.PercentOutput, speed);
    }

  public boolean isBallEntering() {
    return !entranceLineBreak.get();
  }

  public boolean isBallExiting() {
    return !exitLineBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
