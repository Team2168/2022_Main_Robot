// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;



import org.team2168.Constants.MotorSpeeds;
import org.team2168.commands.FireBalls;
import org.team2168.commands.LowerAndRunIntake;
import org.team2168.commands.QueueBallForShot;
import org.team2168.commands.RetractAndStopIntake;
import org.team2168.commands.WaitUntilFireBalls;
import org.team2168.commands.hood.BumpHoodAngleZero;
import org.team2168.commands.hopper.DriveHopperUntilBall;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.indexer.DriveIndexerUntilBall;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.FindTargetAndCenterTurret;
import org.team2168.commands.turret.StopTurret;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Pooper;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Turret;
import org.team2168.subsystems.Shooter.ShooterRPM;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftSideThreeAuto extends SequentialCommandGroup {

    public LeftSideThreeAuto(
      Drivetrain drivetrain,
      IntakeRaiseAndLower intakeRaiseAndLower,
      IntakeRoller intakeRoller,
      Hopper hopper,
      Indexer indexer,
      Hood hood,
      Shooter shooter,
      Turret turret,
      Pooper pooper,
      ColorSensor colorSensor,
      Limelight limelight)
     {
      Paths path = Paths.getInstance();
  
    addCommands(

    new DriveTurretWithLimelight(turret, limelight),
    

      new TwoBall(drivetrain,intakeRaiseAndLower, intakeRoller, hopper, indexer, hood, 
      shooter, turret, pooper, colorSensor, limelight),
      new BumpHoodAngleZero(hood),

      
      PathUtil.getPathCommand(path.path_LineThreeSetupAuto, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY),

      parallel(
      new LowerAndRunIntake(intakeRaiseAndLower, intakeRoller),
      new DriveHopperUntilBall(hopper, () -> MotorSpeeds.HOPPER_SPEED),
      new DriveIndexerUntilBall(indexer, () -> MotorSpeeds.INDEXER_SPEED)
      ),

      new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller));
      
      race(
      PathUtil.getPathCommand(path.path_ReversedThreeSetupAuto, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY)
      );
      
      sequence(
      new QueueBallForShot(hopper, indexer, pooper, colorSensor, intakeRoller),
      new AutoTarmacLine(hood, shooter, limelight),
      new SetPipeline(limelight, Limelight.PIPELINE_TARMAC_LINE),
      new FindTargetAndCenterTurret(turret, limelight),
      new WaitUntilFireBalls(shooter, limelight),
      new FireBalls(shooter, indexer, hopper),
      new FireBalls(shooter, indexer, hopper)
      );

      parallel(
    new SetShooterSpeed(shooter,ShooterRPM.STOP),
    new StopTurret(turret),
    new DriveIndexer(indexer,() -> 0.0)
      );
      


    }

    
}
