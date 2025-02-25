package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerIntake;

public class AutoRollerIntakeCommand extends Command {
    private final RollerIntake m_rollerIntake;

    public AutoRollerIntakeCommand(RollerIntake subystem) {
        m_rollerIntake = subystem;
        addRequirements(m_rollerIntake);
    }

    @Override
    public void execute() {
        m_rollerIntake.RollerIntake();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

}
