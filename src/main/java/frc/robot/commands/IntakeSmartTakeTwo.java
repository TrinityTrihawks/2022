// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BeamState;
import frc.robot.Constants.ShootyBitsConstants;
import frc.robot.subsystems.IntakeBits;

public class IntakeSmartTakeTwo extends CommandBase {
  private IntakeBits intakeBits;
  /** Creates a new IntakeSmarter. */
  public IntakeSmartTakeTwo(IntakeBits intakeBits) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeBits.getAsSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeBits.setIntakeVoltage(ShootyBitsConstants.kIntakeRunSpeed);
    if (intakeBits.getHighBeamState() == BeamState.OPEN) {
      intakeBits.setMiddleVoltage(ShootyBitsConstants.kMiddleRunSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeBits.setIntakeVoltage(0);
    intakeBits.setArmVoltage(0);
    intakeBits.setMiddleVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
