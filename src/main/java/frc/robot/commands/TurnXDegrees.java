// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnXDegrees extends CommandBase {
    private final Drivetrain drivetrain;
    private double degreesToTurn;
    private double initialDegrees;
    private boolean finished = false;
    private Timer timer = new Timer();

    /** Creates a new TurnXDegrees. */
    public TurnXDegrees(Drivetrain drivetrain, double degrees) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.degreesToTurn = degrees;
        addRequirements(this.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialDegrees = drivetrain.getHeading();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (finished) {
            drivetrain.drive(0, 0, 0, false);
        } else if (Math.signum(degreesToTurn) == -1) {
            drivetrain.drive(0, 0, -0.1, true);
        } else if (Math.signum(degreesToTurn) == 1) {
            drivetrain.drive(0, 0, 0.1, true);
        }
        if (initialDegrees - drivetrain.getHeading() < 0) {
            finished = true;
        }
        if ((timer.get() * 2) % 1 == 0) {
            // System.out.println(initialDegrees + ", "+ drivetrain.getHeading()+", " + degreesToTurn);
            // System.out.println(drivetrain.getPose().getX() + "," +
            //         drivetrain.getPose().getY() + "," +
            //         drivetrain.getPose().getRotation().getDegrees());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
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
