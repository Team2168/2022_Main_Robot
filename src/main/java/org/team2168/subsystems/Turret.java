// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team2168.CanDigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private CanDigitalInput hallEffectSensor;
  //the name below was by Conor's request
  private TalonFX turtle;
  private static Turret instance = null;
  private final int TICKS_PER_REV = 2048;

  public Turret() {
    //0 is a placeholdeor, move to constants later
    turtle = new TalonFX(0);
    hallEffectSensor = new CanDigitalInput(turtle);

    turtle.configFactoryDefault();
  }

  public static Turret getInstance() {
    if(instance == null) 
      instance = new Turret();
    return instance;
  }

  public boolean isTurretAtZero() {
    return hallEffectSensor.isFwdLimitSwitchClosed();
  }

  /**
   * Rotates the turret to a position
   * @param rotation Should be between -1 and 1
   */
  public void rotateTurret(double rotation) {
    turtle.set(ControlMode.Position, rotation * TICKS_PER_REV);
  }

  public double position() {
    return turtle.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
