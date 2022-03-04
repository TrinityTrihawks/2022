// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShootyBitsConstants;
import frc.robot.subsystems.ShooterBits;

public class ShootLowGoal extends CommandBase {

  private final ShooterBits shooterBits;
  private static final boolean OPEN = true;
  private static final boolean CLOSED = false;

  private boolean shouldBeRunningMiddleWheel = false;
  private boolean shouldBeRunningShooterWheel = false;

  /** Creates a new ShootLowGoal. */
  public ShootLowGoal(ShooterBits shooterBits) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterBits = shooterBits;
    addRequirements((Subsystem) shooterBits);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateState();
    if (shouldBeRunningShooterWheel) {
      runShooterWheel();
    } else {
      stopShooterWheel();
    }
    if (shouldBeRunningMiddleWheel) {
      runMiddleWheel();
    } else {
      stopMiddleWheel();
    }

  }

  private void updateState() {
    if (shooterBits.getHighBeamState() == OPEN && shooterBits.getMidBeamState() == CLOSED) {
      // ball on shooter wheel, nothing on mid
      // run shooter until shot
      shouldBeRunningShooterWheel = true;

    } else if (shooterBits.getMidBeamState() == OPEN && shooterBits.getHighBeamState() == OPEN) {
      // ball on both
      // run higher until shot, then lower until high beam triggered
      shouldBeRunningMiddleWheel = true;
      shouldBeRunningShooterWheel = true;

    } else if (shooterBits.getMidBeamState() == OPEN && shooterBits.getHighBeamState() == OPEN) {
      // nothing on either, run wheels anyway
      shouldBeRunningMiddleWheel = true;
      shouldBeRunningShooterWheel = true;

    }
  }

  private void runShooterWheel() {
    shooterBits.setShooterVoltage(ShootyBitsConstants.kShooterWheelSpeed);
  }

  private void stopShooterWheel() {
    shooterBits.setShooterVoltage(0.0);
  }

  private void runMiddleWheel() {
    shooterBits.setMiddleVoltage(ShootyBitsConstants.kMidWheelSpeed);
  }

  private void stopMiddleWheel() {
    shooterBits.setMiddleVoltage(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shouldBeRunningShooterWheel = false;
    shouldBeRunningMiddleWheel = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
