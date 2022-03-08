package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeBits {
    public void setIntakeVoltage(double percentOutput);

    public void setMiddleVoltage(double percentOutput);

    public void setShooterVoltage(double percentOutput);

    public boolean getMidBeamState();

    public boolean getHighBeamState();

    public Subsystem getAsSubsystem();
}
