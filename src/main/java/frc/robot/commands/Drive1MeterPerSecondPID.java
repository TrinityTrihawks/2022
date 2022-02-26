// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class Drive1MeterPerSecondPID extends CommandBase {
	private Drivetrain drive;

	public Drive1MeterPerSecondPID(Drivetrain drivetrain) {
		drive = drivetrain;
		addRequirements(drive);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		MecanumDriveWheelSpeeds speeds = 
			DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(1, 0, 0));

		speeds.desaturate(1 /*TODO: figure out what to put here*/);

		drive.drive(speeds);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
