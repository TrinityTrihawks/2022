package frc.robot.subsystems;

public interface IntakeBits {
    public void setIntakeVoltage(double percentOutput);
    public void setMidVoltage(double percentOutput);
    public void setShooterVoltage(double percentOutput);

    public boolean getLowBeamState();
    public boolean getMidBeamState();
    public boolean getHighBeamState();
}
