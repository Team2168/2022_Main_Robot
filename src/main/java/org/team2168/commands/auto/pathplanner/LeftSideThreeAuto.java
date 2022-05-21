// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;



import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.StopMechanisms;
import org.team2168.commands.WaitUntilFireBalls;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.commands.shootingpositions.ShootBasedOnDistance;
import org.team2168.commands.shootingpositions.auto.AutoTarmacLine;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.RotateTurret;
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
import org.team2168.subsystems.Shooter.ShooterRPM;
import org.team2168.subsystems.Turret;
import org.team2168.subsystems.Hood.HoodPosition;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
       
      new RotateTurret(turret, 0.0).withTimeout(0.2),
      new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),

     race(
      new DriveTurretWithLimelight(turret, limelight),
      new ShootBasedOnDistance(shooter, hood, limelight),
    
      sequence(
       new HoodToAngle(hood, HoodPosition.AUTO_TARMAC_LINE.position_degrees),
       new SetShooterSpeed(shooter, ShooterRPM.AUTO_TARMAC_LINE),
        new IntakeLower(intakeRaiseAndLower),
     race(
        new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
    PathUtil.getPathCommand(path.path_TwoBallLeft, drivetrain, 
    PathUtil.InitialPathState.DISCARDHEADING
),

      sequence(
        
    new StopMechanisms(hopper, indexer, intakeRoller, drivetrain),
    race(
      PathUtil.getPathCommand(path.path_ReverseTwoBallLeft, drivetrain, 
      PathUtil.InitialPathState.PRESERVEODOMETRY)
    )
      ),

    sequence(
       new WaitUntilFireBalls(shooter, limelight),
       new FireBalls(shooter, indexer, hopper),
       new FireBalls(shooter, indexer, hopper)
    ,
     
     new StopMechanisms(hopper, indexer, intakeRoller, drivetrain),
     
     sequence(
      new IntakeLower(intakeRaiseAndLower),
    
    race(
      new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
        PathUtil.getPathCommand(path.path_LineThreeSetupAuto, drivetrain, 
        PathUtil.InitialPathState.PRESERVEODOMETRY)
      ),

       sequence(
     new StopMechanisms(hopper, indexer, intakeRoller, drivetrain),
      new IntakeRaise(intakeRaiseAndLower).withTimeout(0.1),
      race(
      PathUtil.getPathCommand(path.path_ReversedThreeSetupAuto, drivetrain, 
      PathUtil.InitialPathState.PRESERVEODOMETRY)
      ),

      sequence(
        parallel(
      race(
        new HoodToAngle(hood, HoodPosition.AUTO_TARMAC_LINE.position_degrees),
        new SetShooterSpeed(shooter, ShooterRPM.AUTO_TARMAC_LINE),
      new SetPipeline(limelight, Limelight.PIPELINE_TARMAC_LINE)
      )
        ).withTimeout(0.5),

      new StopMechanisms(hopper, indexer, intakeRoller, drivetrain),
      new WaitUntilFireBalls(shooter, limelight),
      new FireBalls(shooter, indexer, hopper),
      new FireBalls(shooter, indexer, hopper)
         ),

     parallel(
    new SetShooterSpeed(shooter,ShooterRPM.STOP),
    new StopTurret(turret)

     )
)))))));
    }
}
