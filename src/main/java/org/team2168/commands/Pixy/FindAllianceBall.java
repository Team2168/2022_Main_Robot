// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Pixy;

import java.util.ArrayList;

import org.team2168.subsystems.Pixy;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class FindAllianceBall extends CommandBase {
  private Pixy pixy;
  private Alliance ourAlliance;
  public static byte signature_filter;

  private static final byte RED_BALL_SIGNATURE = Pixy2CCC.CCC_SIG1; // these need to match the trained object signatures on the Pixy
  private static final byte BLUE_BALL_SIGNATURE = Pixy2CCC.CCC_SIG2;
  private static final int MAX_TARGETS = 25;

  /** Creates a new FindAllianceBall. */
  public FindAllianceBall(Pixy pixy) {
    this.pixy = pixy;
    addRequirements(pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ourAlliance = DriverStation.getAlliance();
    if(ourAlliance == Alliance.Blue) {
      signature_filter = BLUE_BALL_SIGNATURE;
    } else {
      signature_filter = RED_BALL_SIGNATURE;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int blockCount = pixy.getCCC().getBlocks(false, signature_filter, MAX_TARGETS);
		if (blockCount <= 0) {
			// System.err.println("No block count");
			return;
		}
		ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
		Block largestBlock = null;
		if (blocks == null) {
			// System.err.println("No Blocks");
			return;
		}
		for (Block block : blocks) {
			// if (block.getSignature() == blockSignature) {  //shouldn't need to filter since we asked for that when requesting the blocks
				if (largestBlock == null) {
					largestBlock = block;
				} else if (block.getWidth() > largestBlock.getWidth()) {
					largestBlock = block;
				}
			// }
		}

    // largestBlock.getAngle();
    // largestBlock.getHeight();
    // largestBlock.getWidth();
    // largestBlock.getX();
    // largestBlock.getY();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}