// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.robot.Constants.BeamState;
import static frc.robot.Constants.ShootyBitsConstants;
import frc.robot.subsystems.IntakeBits;

/**
 * Smart intake command
 * 
 * <p>
 * sub-delegates actual execution to inner classes depending on whether
 * there are one or no balls in the robot
 */
public class IntakeSmart extends CommandBase {

	private class IntakeUpperSmart extends CommandBase {
		private IntakeBits intakeBits;

		private boolean lowerBeamHasBeenTriggered = false;

		public IntakeUpperSmart(IntakeBits intake) {
			intakeBits = intake;
			addRequirements(intakeBits.getAsSubsystem());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			updateState();
			if (shouldBeRunning()) {
				runWheels();
			}
			System.out.println(this + ": low trigger " + lowerBeamHasBeenTriggered);
		}

		private void updateState() {
			if (intakeBits.getLowBeamState() == BeamState.OPEN) {
				lowerBeamHasBeenTriggered = true;
			}
		}

		private boolean shouldBeRunning() {
			return !shouldStop();
		}

		private boolean shouldStop() {
			return intakeBits.getLowBeamState() == BeamState.CLOSED 
				&& lowerBeamHasBeenTriggered;
		}

		private void runWheels() {
			intakeBits.setIntakeVoltage(ShootyBitsConstants.kIntakeRunSpeed);
			intakeBits.setMiddleVoltage(ShootyBitsConstants.kMiddleRunSpeed);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			stopWheels();
		}

		private void stopWheels() {
			intakeBits.setIntakeVoltage(0);
			intakeBits.setMiddleVoltage(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return shouldStop();
		}
	}

	private class IntakeLowerSmart extends CommandBase {
		private IntakeBits intakeBits;
		private Timer timer = new Timer();

		public IntakeLowerSmart(IntakeBits intake) {
			intakeBits = intake;
			addRequirements(intakeBits.getAsSubsystem());
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			if (shouldRunIntake()) {
				runIntake();
			} else {
				timer.start();
			}
		}

		private boolean shouldRunIntake() {
			return intakeBits.getLowBeamState() == BeamState.CLOSED;
		}

		private void runIntake() {
			intakeBits.setIntakeVoltage(ShootyBitsConstants.kIntakeRunSpeed);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			stopIntake();
		}

		private void stopIntake() {
			intakeBits.setIntakeVoltage(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return (!shouldRunIntake()) && timer.hasElapsed(0.5);
		}
	}

//#region IntakeSmart

	private IntakeBits intakeBits;
	private Command delegatedCmd;

	public IntakeSmart(IntakeBits intake) {
		intakeBits = intake;
		addRequirements(intakeBits.getAsSubsystem());
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.print(this + ": delegating ");
		if (intakeBits.getHighBeamState() == BeamState.OPEN) {
			delegatedCmd = new IntakeLowerSmart(intakeBits);
		} else {
			delegatedCmd = new IntakeUpperSmart(intakeBits);
		}
		System.out.println(delegatedCmd);
		CommandScheduler.getInstance().schedule(delegatedCmd);
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
		return delegatedCmd.isFinished();
	}
//#endregion
}
