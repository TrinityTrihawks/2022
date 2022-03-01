// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeBits;

public class Intake1Ball extends CommandBase {

	private class Intake1Lower extends CommandBase {

		private IntakeBits intakeBits;

		private boolean isPastIntakeWheel = false;
		private boolean isInPlace = false;

		public Intake1Lower(IntakeBits intakeBits) {

			this.intakeBits = intakeBits;

			addRequirements((Subsystem) this.intakeBits);
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {

		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {

			updateBooleans();

			if (shouldBeIntaking()) {
				runIntakeWheel();
			} else {
				stopIntakeWheel();
			}

			runMiddleWheel();

		}

		private void updateBooleans() {
			isPastIntakeWheel = !intakeBits.getLowBeamState();
			isInPlace = intakeBits.getMidBeamState();
		}

		private boolean shouldBeIntaking() {
			return isPastIntakeWheel;
		}

		private void runIntakeWheel() {
			intakeBits.setIntakeVoltage(1);
		}

		private void stopIntakeWheel() {
			intakeBits.setIntakeVoltage(0);
		}

		private void runMiddleWheel() {
			intakeBits.setMidVoltage(1);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			stopMiddleWheel();
		}

		private void stopMiddleWheel() {
			intakeBits.setMidVoltage(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return isInPlace;
		}
	}

	private class Intake1Upper extends CommandBase {
		
		private IntakeBits intakeBits;

		public Intake1Upper(IntakeBits intakeBits) {

			this.intakeBits = intakeBits;

			addRequirements((Subsystem) this.intakeBits);
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {

		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
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

//#region abstraction
	
private IntakeBits intakeBits;

	public Intake1Ball(IntakeBits intakeBits) {

		this.intakeBits = intakeBits;

		addRequirements((Subsystem) this.intakeBits);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (oneBallLoaded()) {
			CommandScheduler.getInstance().schedule(new Intake1Lower(intakeBits));
		} else {
			CommandScheduler.getInstance().schedule(new Intake1Upper(intakeBits));
		}
	}

	private boolean oneBallLoaded() {
		return intakeBits.getHighBeamState();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
//#endregion
}