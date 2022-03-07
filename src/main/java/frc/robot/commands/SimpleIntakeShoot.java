// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootyBits;

public class SimpleIntakeShoot extends CommandBase {
  private ShootyBits shooter;
  private BooleanSupplier shouldRun;

  public SimpleIntakeShoot(ShootyBits bits, BooleanSupplier shouldRun) {

    shooter = bits;
    this.shouldRun = shouldRun;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("==============simple intake shoot init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shouldRun.getAsBoolean()) {
      shooter.setIntakeVoltage(-0.25);
      shooter.setShooterVoltage(1);
      //System.out.println("running intake shooter");
    }
    shooter.setMiddleVoltage(1);
    //System.out.println("simple intake shoot exec");
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
