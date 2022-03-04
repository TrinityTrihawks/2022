// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.JoystickConstants;
import frc.robot.commands.Drive5ftInAutoOdo;
import frc.robot.commands.Drive5ftSideways;
import frc.robot.commands.DriveDoubleJoystick;
import frc.robot.commands.DriveSingleJoystick;
import frc.robot.commands.DriveZero;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SimpleIntakeShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootyBits;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final ShootyBits shootyBits = ShootyBits.getInstance();

    // Joysticks
    private final ZeroableJoystick mainJoystick = new ZeroableJoystick(JoystickConstants.kMainJoystickPort, "Thor");
    private final ZeroableJoystick auxJoystick = new ZeroableJoystick(JoystickConstants.kAuxJoystickPort,
            "Loki (balthazar)"); // balthazar
    private final JoystickButton zeroButton = new JoystickButton(mainJoystick, JoystickConstants.kZeroButtonId);
    private final JoystickButton switchDriveModeButton = new JoystickButton(mainJoystick,
            JoystickConstants.kSwitchDriveModeButtonId);
    private final JoystickButton shootButton = new JoystickButton(mainJoystick, 1);

    // Commands
    private DriveSingleJoystick singleDefault = new DriveSingleJoystick(
            drivetrain,
            () -> -mainJoystick.getZeroedY(),
            () -> -mainJoystick.getZeroedX(),
            () -> mainJoystick.getZeroedTwist(),
            () -> mainJoystick.getThrottle());

    private DriveDoubleJoystick doubleDefault = new DriveDoubleJoystick(
            drivetrain,
            () -> auxJoystick.getZeroedX(),
            () -> mainJoystick.getZeroedX(),
            () -> auxJoystick.getZeroedY(),
            () -> mainJoystick.getZeroedY(),
            () -> mainJoystick.getThrottle());

    private SimpleIntakeShoot shoot = new SimpleIntakeShoot(
        shootyBits,
        () -> shootButton.debounce(0.5).getAsBoolean()
    );
    
    private final NetworkTable subtable;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        subtable = NetworkTableInstance.getDefault().getTable("RobotContainer");

        // Set the scheduler to log Shuffleboard events for command initialize,
        // interrupt, finish
        CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker(
                "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker(
                "Command interrupted", command.getName(), EventImportance.kHigh));
        CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker(
                "Command finished", command.getName(), EventImportance.kNormal));

        configureDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        // Drivetrain default
        drivetrain.setDefaultCommand(singleDefault);
        shootyBits.setDefaultCommand(shoot);
    }

    /**
     * This method defines the button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        bindZeroButton();
        bindSwitchDriveModeButton();
    }

    /**
     * Isaac helped more than Luca
     */
    private void bindZeroButton() {
        Runnable zero = () -> {
            mainJoystick.zero();
            auxJoystick.zero();
        };
        zeroButton.debounce(0.5).whenActive(zero, drivetrain);
    }

    private void bindSwitchDriveModeButton() {
        Runnable switchDriveMode = () -> {
            if (drivetrain.getDefaultCommand() == singleDefault) {
                drivetrain.setDefaultCommand(doubleDefault);
            } else {
                drivetrain.setDefaultCommand(singleDefault);
            }
        };
        switchDriveModeButton.debounce(0.5).whenActive(switchDriveMode, drivetrain);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command resetGyro = new ResetGyro(drivetrain, DriveConstants.kGyroResetWaitTime);
        return resetGyro.andThen(new Drive5ftInAutoOdo(drivetrain)).andThen(new DriveZero(drivetrain));
    }
}
