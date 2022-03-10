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
import static frc.robot.Constants.ShootyBitsConstants;

import frc.robot.Constants.XboxInterface;
import frc.robot.commands.Drive5ftInAutoOdo;
import frc.robot.commands.DriveDoubleJoystick;
import frc.robot.commands.DriveSingleJoystick;
import frc.robot.commands.DriveZero;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootyBits;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private final XboxInterface xboxInterface = new XboxInterface();

    private final ZeroableJoystick rightJoystick = new ZeroableJoystick(JoystickConstants.kRightJoystickPort, "Thor"); // Thor
    private final ZeroableJoystick leftJoystick = new ZeroableJoystick(JoystickConstants.kLeftJoystickPort, "Loki"); // Loki
    private final XboxController xboxController = new XboxController(JoystickConstants.kXboxControllerPort); // Odin

    // Thor and Loki
    private final JoystickButton zeroButton = new JoystickButton(rightJoystick, JoystickConstants.kZeroButtonId);
    private final JoystickButton switchDriveModeButton = new JoystickButton(rightJoystick,
            JoystickConstants.kSwitchDriveModeButtonId);

    // Odin
    private final JoystickButton intakeVacuumButton = new JoystickButton(xboxController, xboxInterface.a());
    private final JoystickButton intakeSpitButton = new JoystickButton(xboxController, xboxInterface.x());
    private final JoystickButton runShooterButton = new JoystickButton(xboxController, xboxInterface.b());
    private final Trigger boostTrigger = new Trigger(() -> xboxController.getRawAxis(xboxInterface.lt()) > 0.9);

    // Commands
    private DriveSingleJoystick singleDefault = new DriveSingleJoystick(
            drivetrain,
            () -> -rightJoystick.getZeroedY(),
            () -> -rightJoystick.getZeroedX(),
            () -> rightJoystick.getZeroedTwist(),
            () -> rightJoystick.getThrottle());

    private DriveDoubleJoystick doubleDefault = new DriveDoubleJoystick(
            drivetrain,
            () -> leftJoystick.getZeroedX(),
            () -> rightJoystick.getZeroedX(),
            () -> leftJoystick.getZeroedY(),
            () -> rightJoystick.getZeroedY(),
            () -> rightJoystick.getThrottle());

    private StartEndCommand runIntake = new StartEndCommand(
            () -> {
                shootyBits.setIntakeVoltage(ShootyBitsConstants.kIntakeRunSpeed);
                shootyBits.setMiddleVoltage(ShootyBitsConstants.kMiddleRunSpeed);
            },

            () -> {
                shootyBits.setIntakeVoltage(0);
                shootyBits.setMiddleVoltage(0);
            },

            shootyBits);
    private StartEndCommand runIntakeReverse = new StartEndCommand(
            () -> {
                shootyBits.setIntakeVoltage(-ShootyBitsConstants.kIntakeRunSpeed);
                shootyBits.setMiddleVoltage(-ShootyBitsConstants.kMiddleRunSpeed);
            },

            () -> {
                shootyBits.setIntakeVoltage(0);
                shootyBits.setMiddleVoltage(0);
            },

            shootyBits);
    private StartEndCommand runShooter = new StartEndCommand(
            () -> shootyBits.setShooterVoltage(ShootyBitsConstants.kShooterRunSpeed),
            () -> shootyBits.setShooterVoltage(0),
            shootyBits);
    private StartEndCommand runShooterSlow = new StartEndCommand(
            () -> shootyBits.setShooterVoltage(ShootyBitsConstants.kShooterSlowSpeed),
            () -> shootyBits.setShooterVoltage(0),
            shootyBits);

    private ParallelCommandGroup runAll = new ParallelCommandGroup(runIntake, runShooter);

    private ParallelCommandGroup runAllWithSlowShooter = new ParallelCommandGroup(runIntake, runShooterSlow);

    private final NetworkTable subtable;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        subtable = NetworkTableInstance.getDefault().getTable("RobotContainer");
        subtable.getInstance();
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

        configureXboxButtons();
    }

    private void configureXboxButtons() {
        intakeVacuumButton.whileActiveOnce(runIntake);
        intakeSpitButton.whileActiveOnce(runIntakeReverse);
        runShooterButton.whileActiveOnce(runShooter);

    }

    private void configureDefaultCommands() {
        // Drivetrain default
        drivetrain.setDefaultCommand(singleDefault);
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
            rightJoystick.zero();
            leftJoystick.zero();
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
