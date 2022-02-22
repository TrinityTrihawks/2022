package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeUntilLimitSwitch extends CommandBase {

    private final Intake intake;
    private boolean finished = false;

    /**
     * Creates a new RunIntakeUntilLimitSwitch.
     *
     * @param intake The subsystem used by this command.
     */
    public RunIntakeUntilLimitSwitch(Intake subsystem) {
        intake = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeVoltage(0.2);
    }

    @Override
    public void execute() {
        if (intake.getLimitSwitchState() == true) {
            intake.setIntakeVoltage(0.0);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
