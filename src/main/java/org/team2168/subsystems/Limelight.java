/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Limelight inst;

   NetworkTable networkTable;
   NetworkTableEntry tx;
   NetworkTableEntry ty;
   NetworkTableEntry ta;
   NetworkTableEntry ledMode;
   NetworkTableEntry camMode;
   NetworkTableEntry camtran;
   NetworkTableEntry pipeline;

  private boolean variablesInstantiated;

  /*
   * Public so they can be used elsewhere; idk if this is good design but I need
   * it for right now because not every ll pipeline is correlated to a shooter
   * speed anymore and I don't want to overhaul the whole subsystem :/
   */
  //TODO create new pipelines
  public static final int PIPELINE_FORWARD_BLUE = 0;
  public static final int PIPELINE_FORWARD_RED = 2;
  public static final int PIPELINE_BACK_TRENCH_BLUE = 1;
  public static final int PIPELINE_BACK_TRENCH_RED = 3;
  public static final int PIPELINE_DRIVER_VIEW = 4;
  public static final int PIPELINE_DRIVE_WITH_LIMELIGHT = 9;

  private boolean isLimelightEnabled;

  /*
   * private Limelight() { // set up limelight limelight.setCamMode(0);
   * limelight.enableVisionProcessing(true); limelight.setLedMode(1);
   * limelight.setPipeline(PIPELINE_DRIVE_WITH_LIMELIGHT); isLimelightEnabled =
   * false; }
   */

  /**
   * @return an instance of the Intake Subsystem
   */
  public static Limelight getInstance() {
    if (inst == null) {
      inst = new Limelight();
    }
    return inst;
  }

  /**
   * Sets the mode of the camera
   * 
   * @param doVisionProcessing Whether or not to do vision processing. If set to
   *                           false, the Limelight will act as a driver camera
   */
  public void enableVisionProcessing(boolean doVisionProcessing) {
    if (!connectionEstablished())
      return;

    else if (!variablesInstantiated)
      instantiateLocalVariables();

    camMode.setNumber(doVisionProcessing ? 0 : 1);
  }

  /**
   * Returns the target bearing
   * 
   * @return is a double from -27.0 to 27.0
   */
  public double getPositionX() {
    return tx.getDouble(0.0);
  }

  /**
   * Returns the target bearing
   * 
   * @return is a double from -20.5 to 20.5 degrees
   */
  public double getPositionY() {
    return ty.getDouble(0.0);
  }

  public void enableLimelight() {
    // limelight.setCamMode(0);
    enableVisionProcessing(true);
    setLedMode(0); // set to current pipeline setting
    setPipeline(PIPELINE_DRIVE_WITH_LIMELIGHT);
    isLimelightEnabled = true;
  }

  public void setPipeline(int pipeline) {
    setPipeline(pipeline);
  }

  public int getPipeline() {
    return getPipeline();
  }

  public void pauseLimelight() {
    setCamMode(1);
    enableVisionProcessing(false);
    setLedMode(1); // force off
    isLimelightEnabled = false;

  }

  public boolean isLimelightEnabled() {
    return isLimelightEnabled;
  }

  /**
   * Sets the LED mode
   * 
   * @param ledNumber is an int from 0 to 3
   *                  <ul>
   *                  <li>0 - use the LED Mode set in the current pipeline</li>
   *                  <li>1 - force off</li>
   *                  <li>2 - force blink</li>
   *                  <li>3 - force on</li>
   *                  </ul>
   */
  public void setLedMode(int ledNumber) {
    setLedMode(ledNumber);
  }

  private boolean connectionEstablished() {
    // return this.networkTable.containsKey("tx");
    return !(this.networkTable.getEntry("tx") == null);
  }

  private void instantiateLocalVariables() {
    SmartDashboard.putBoolean("IsLimeLightPresent", true);

    // Variables to get data from Limelight
    tx = networkTable.getEntry("tx");
    ta = networkTable.getEntry("ta");
    camtran = networkTable.getEntry("camtran");

    // Variables to set data on Limelight
    ledMode = networkTable.getEntry("ledMode");
    camMode = networkTable.getEntry("camMode");
    pipeline = networkTable.getEntry("pipeline");

    // Sets the camera controls
    ledMode.setNumber(0);
    camMode.setNumber(1);
    pipeline.setNumber(0);

    this.variablesInstantiated = true;
  }

  /**
   * Returns the target's area, measured by the percentage of the frame it takes
   * up
   * 
   * @return is a double from 0.0 to 100.0
   */
  public double getTargetArea() {
    if (this.connectionEstablished() && this.variablesInstantiated) {
      return ta.getDouble(0.0);
    } else if (this.connectionEstablished() && !this.variablesInstantiated) {
      this.instantiateLocalVariables();
      return ta.getDouble(0.0);
    } else {
      return 0.0;
    }
  }

  /**
   * Sets the limelight camera mode
   * 
   * @param camModeNumber an integer for the camera mode
   *                      <ul>
   *                      <li>0 - Vision processing</li>
   *                      <li>1 - Driver camera (More exposure, no vision
   *                      processing)</li>
   */
  private void setCamMode(int camModeNumber) {
    if (camModeNumber >= 0 && camModeNumber <= 2) {
      if (this.connectionEstablished() && this.variablesInstantiated) {
        camMode.setNumber(camModeNumber);
      } else if (this.connectionEstablished() && !this.variablesInstantiated) {
        this.instantiateLocalVariables();
        camMode.setNumber(camModeNumber);
      }
      // else
      // {
      // System.out.println("Connection to Limelight not established. Check ethernet
      // connectors.");
      // }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void initDefaultCommand() {
  // Set the default command for a subsystem here.
  // setDefaultCommand(new MySpecialCommand());
  // setDefaultCommand(new UpdatePipeline());
  // }
}
