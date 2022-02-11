package frc.robot.commands.intake;

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
        intake.vacuum();
    }

    @Override
    public void execute() {
        if (intake.getLimitSwitchState() == true) {
            intake.off();
            finished = true;
        }
        // System.out.println(intake.getLimitSwitchState());
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
