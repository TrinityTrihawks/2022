package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.BeamState;

public interface IntakeBits {
    public void setIntakeVoltage(double percentOutput);

    public void setMiddleVoltage(double percentOutput);

    public BeamState getLowBeamState();

    public BeamState getHighBeamState();

    public Subsystem getAsSubsystem();
}
