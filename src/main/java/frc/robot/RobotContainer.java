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
import frc.robot.commands.DriveDoubleJoystick;
import frc.robot.commands.DriveSingleJoystick;
import frc.robot.commands.DriveZero;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.DriveSingleJoystick;
import frc.robot.commands.DriveDoubleJoystick;
import frc.robot.commands.RunIntakeUntilLimitSwitch;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
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

	// Joysticks
	private final ZeroableJoystick mainJoystick = new ZeroableJoystick(JoystickConstants.kMainJoystickPort, "Thor");
	private final ZeroableJoystick auxJoystick = new ZeroableJoystick(JoystickConstants.kAuxJoystickPort,
			"Loki (balthazar)"); // balthazar
	private final JoystickButton zeroButton = new JoystickButton(mainJoystick, JoystickConstants.kZeroButtonId);
	private final JoystickButton switchDriveModeButton = new JoystickButton(mainJoystick,
			JoystickConstants.kSwitchDriveModeButtonId);

	// Commands
	DriveSingleJoystick singleDefault = new DriveSingleJoystick(
			drivetrain,
			() -> -mainJoystick.getZeroedY(),
			() -> -mainJoystick.getZeroedX(),
			() -> mainJoystick.getZeroedTwist(),
			() -> mainJoystick.getThrottle());

	DriveDoubleJoystick doubleDefault = new DriveDoubleJoystick(
			drivetrain,
			() -> -auxJoystick.getZeroedY(),
			() -> -mainJoystick.getZeroedY(),
			() -> -auxJoystick.getZeroedX(),
			() -> -mainJoystick.getZeroedX(),
			() -> mainJoystick.getThrottle());


	private final NetworkTable subtable; //?

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
		drivetrain.setDefaultCommand(doubleDefault);
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
				System.out.print("===== SWITCHED MODE TO DOUBLE JOYSTICK =====");
			} else {
				drivetrain.setDefaultCommand(singleDefault);
				System.out.print("===== SWITCHED MODE TO SINGLE JOYSTICK =====");
			}
		};
		switchDriveModeButton.debounce(0.5).whenActive(switchDriveMode, drivetrain);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 */
	public Command getAutonomousCommand() {
		Command resetGyro = new ResetGyro(drivetrain, DriveConstants.kGyroResetWaitTime);
		return resetGyro.andThen(new Drive5ftInAutoOdo(drivetrain)).andThen(new DriveZero(drivetrain));
	}

}
