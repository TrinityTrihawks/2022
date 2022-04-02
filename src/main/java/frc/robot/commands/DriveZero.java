// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveZero extends CommandBase {

	private Drivetrain drive;
	private boolean hasBeenInt = false;

	public DriveZero(Drivetrain drivetrain) {
		drive = drivetrain;
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println(this + ": beginning to not drive");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.println(this + ": not driving");
		drive.drive(0, 0, 0, false);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		hasBeenInt = interrupted;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}