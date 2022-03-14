// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.BeamState;

/** Shooter Interface. */
public interface ShooterBits {
    public void setShooterVoltage(double percentOutput);

    public void setMiddleVoltage(double percentOutput);

    public BeamState getLowBeamState();

    public BeamState getHighBeamState();

    public Subsystem getAsSubsystem();
}