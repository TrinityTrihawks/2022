// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DriveSingleJoystick;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // Commands

  // Joysticks
  private final Joystick mainJoystick = new Joystick(JoystickConstants.kMainJoystickPort);
  //private final Joystick auxJoystick = new Joystick(JoystickConstants.kAuxJoystickPort);

  final NetworkTable subtable;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    subtable = NetworkTableInstance.getDefault().getTable("RobotContainer");
    
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
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
      drivetrain.setDefaultCommand(new DriveSingleJoystick(
        drivetrain,
        () -> mainJoystick.getX(),
        () -> mainJoystick.getY(),
        () -> mainJoystick.getTwist(),
        () -> mainJoystick.getThrottle()
      ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
