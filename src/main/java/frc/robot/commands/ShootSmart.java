// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
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
public class ShootSmart extends ProxyScheduleCommand {
    private final static double kShooterShutdownTime = 1;
    private final static double kShooterWarmupTime = 1.2;

    private ShootSmart(ShooterBits shooter) {
        super(getCommand(shooter));
    }

    public static Command create(ShooterBits shooter) {
        return new ProxyScheduleCommand(getCommand(shooter));
    }

    private static Command getCommand(ShooterBits shooter) {

        if (shooter.getHighBeamState() == BeamState.CLOSED && shooter.getLowBeamState() == BeamState.OPEN) {

            return new SequentialCommandGroup(
                genStartShooter(shooter),

                genShoot1(shooter),
                
                genStopShooter(shooter),

                new PrintCommand("ShootSmart: done shooting one")
            );

        } else if (shooter.getHighBeamState() == BeamState.OPEN && shooter.getLowBeamState() == BeamState.OPEN) {

            return new SequentialCommandGroup(
                genStartShooter(shooter),

                genShoot1(shooter),
                new WaitCommand(0.5),
                genShoot1(shooter),

                genStopShooter(shooter)
            );

        }
        return new InstantCommand();
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
    private static Command genStartShooter(ShooterBits shooter) {
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
    private static Command genShoot1(ShooterBits shooter) {
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
    private static Command genStopShooter(ShooterBits shooter) {
        return new SequentialCommandGroup(
            new WaitCommand(kShooterShutdownTime),
            new InstantCommand(
                () -> shooter.setShooterVoltage(0),
                shooter.getAsSubsystem())
        );
    }
}
