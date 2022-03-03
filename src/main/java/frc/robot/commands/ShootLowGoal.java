// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterBits;

public class ShootLowGoal extends CommandBase {

  private final ShooterBits shooterBits;
  private static final boolean OPEN = true;
  private static final boolean CLOSED = false;

  /** Creates a new ShootLowGoal. */
  public ShootLowGoal(ShooterBits shooterBits) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterBits = shooterBits;
    addRequirements((Subsystem) shooterBits);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shooterBits.getHighBeamState() == OPEN && shooterBits.getMidBeamState() == CLOSED) {
      // ball on shooter wheel, nothing on mid
      shooterBits.setShooterVoltage(0.8);

    } else if (shooterBits.getMidBeamState() == OPEN && shooterBits.getHighBeamState() == OPEN) {
      // ball on both

    } else if (shooterBits.getMidBeamState() == OPEN && shooterBits.getHighBeamState() == OPEN) {
      // nothing on either, run wheels anyway
    }

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
