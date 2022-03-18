// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BeamState;
import frc.robot.Constants.ShootyBitsConstants;
import frc.robot.subsystems.ShooterBits;

/**
 * Smart shoot command
 * 
 * <p>
 * waits for the shooter wheel to get to speed,
 * then branches:
 * 
 * <pre>
- if only one ball, bring it up
- if two, 
    > shoot 1
    > intake
    > shoot 1
 * </pre>
 * 
 * all the while running the shooter wheel
 */
public class ShootSmart extends CommandBase {
    private final double kShooterShutdownTime = 1;
    private final double kShooterWarmupTime = 1;
    private ShooterBits shooter;
    private Command delegatedCmd;

    public ShootSmart(ShooterBits shooter) {
        this.shooter = shooter;
        addRequirements(shooter.getAsSubsystem());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        delegatedCmd = getCommand();
        CommandScheduler.getInstance().schedule(delegatedCmd);
    }

    private Command getCommand() {

        if (shooter.getHighBeamState() == BeamState.OPEN && shooter.getLowBeamState() == BeamState.CLOSED) {

            return new SequentialCommandGroup(
                genStartShooter(),

                genShoot1(),
                
                genStopShooter()
            );

        } else if (shooter.getHighBeamState() == BeamState.OPEN && shooter.getLowBeamState() == BeamState.OPEN) {

            return new SequentialCommandGroup(
                genStartShooter(),

                genShoot1(),
                new WaitCommand(0.5),
                genShoot1(),

                genStopShooter()
            );

        }
        return null;
    }

    /**
     * creates a command to start the shooter
     * and wait for it to get up to speed
     * 
     * <p>
     * in my (Xavier's) understanding,
     * a single command instance could only run once.
     * if I'm wrong, feel free to introduce a
     * local variable to getCommand()
     */
    private Command genStartShooter() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> shooter.setShooterVoltage(ShootyBitsConstants.kShooterRunSpeed),
                shooter.getAsSubsystem()),
            new WaitCommand(kShooterWarmupTime)
        );
    }

    /**
     * this method creates a simple command to
     * shoot the upper ball.
     * 
     * <p>
     * in my (Xavier's) understanding,
     * a single command instance could only run once.
     * if I'm wrong, feel free to introduce a
     * local variable to getCommand()
     */
    private Command genShoot1() {
        return new StartEndCommand(
            () -> shooter.setMiddleVoltage(ShootyBitsConstants.kMiddleRunSpeed),
            () -> shooter.setMiddleVoltage(0),
            shooter.getAsSubsystem()
        ).withTimeout(0.5);
    }

    /**
     * creates a command to start the shooter
     * and wait for it to get up to speed
     * 
     * <p>
     * in my (Xavier's) understanding,
     * a single command instance could only run once.
     * if I'm wrong, feel free to introduce a
     * local variable to getCommand()
     */
    private Command genStopShooter() {
        return new SequentialCommandGroup(
            new WaitCommand(kShooterShutdownTime),
            new InstantCommand(
                () -> shooter.setShooterVoltage(0),
                shooter.getAsSubsystem())
        );
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
}
