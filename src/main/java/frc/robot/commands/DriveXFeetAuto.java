// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveXFeetAuto extends CommandBase {
  private final Drivetrain drivetrain;
  private boolean finished = false;
  private double feetToDrive = 0;
  private Timer timer = new Timer();

  public DriveXFeetAuto(Drivetrain drivetrain, double feet) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
    this.feetToDrive = feet;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (finished) {
      drivetrain.drive(0, 0, 0, false);
    } else if (Math.signum(feetToDrive) == 1) {
      drivetrain.drive(0.2, 0, 0, false);
  } else if (Math.signum(feetToDrive) == -1) {
    drivetrain.drive(-0.2, 0, 0, false);
}
  if (drivetrain.getPose().getX() >= feetToDrive * DriveConstants.kFeetToMetersConversionFactor) { // meters
      finished = true;
  }
  if ((timer.get() * 2) % 1 == 0) {
      // System.out.println(drivetrain.getPose().getX() + "," +
      //                    drivetrain.getPose().getY() + "," +
      //                    drivetrain.getPose().getRotation().getDegrees());
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isStopped() && finished;
  }

  private boolean isStopped() {
    MecanumDriveWheelSpeeds speeds = drivetrain.getCurrentWheelSpeeds();
    return Math.abs(speeds.frontLeftMetersPerSecond) +
        Math.abs(speeds.frontRightMetersPerSecond) +
        Math.abs(speeds.rearLeftMetersPerSecond) +
        Math.abs(speeds.rearRightMetersPerSecond) == 0;
  }
}
