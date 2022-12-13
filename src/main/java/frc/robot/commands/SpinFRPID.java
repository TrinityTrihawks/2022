// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * simple pid loop testing command
 * 
 * <p>
 * takes kp, ki, target, and error tolerances from
 * smartdashboard and runs the front right wheel at
 * the target (in RPM) via those kp & ki
 */

public class SpinFRPID extends CommandBase {

    private Drivetrain drivetrain;
    private PIDController controller = new PIDController(0, 0, 0);

    public SpinFRPID(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // SmartDashboard.putNumber("kp", 0);
        // SmartDashboard.putNumber("ki", 0);
        // SmartDashboard.putNumber("velErrTolerance", 0);
        // SmartDashboard.putNumber("target", 0);
        // SmartDashboard.putNumber("accErrTolerance", 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double kp = SmartDashboard.getNumber("kp", 0);
        double ki = SmartDashboard.getNumber("ki", 0);
        double target = SmartDashboard.getNumber("target", 0);
        double velErrTolerance = SmartDashboard.getNumber("velErrTolerance", 0);
        double accErrTolerance = SmartDashboard.getNumber("accErrTolerance", 0);

        controller.setP(kp);
        controller.setI(ki);
        controller.setTolerance(velErrTolerance, accErrTolerance);
        controller.setSetpoint(target);
        drivetrain.setFR(controller.calculate(drivetrain.getFRRPM()) / 5600 +
                controller.getSetpoint() * 0.9); // hacky feedforward
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDriveMotorControllersVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
