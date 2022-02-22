// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive5ftSideways extends CommandBase {
	private Drivetrain drive;
	private boolean finished = false;

	public Drive5ftSideways(Drivetrain drivetrain) {
		drive = drivetrain;
		addRequirements(drive);

	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drive.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!finished) {
			drive.drive(0, 0.2, 0, true);
		}
		if (drive.getPose().getY() >= 1.524) {
			finished = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.drive(0, 0, 0, false);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
