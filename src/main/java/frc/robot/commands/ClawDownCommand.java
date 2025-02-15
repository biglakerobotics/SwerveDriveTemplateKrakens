package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ClawDownCommand extends Command {
    private final Claw m_claw;
    public ClawDownCommand(Claw subsystem) {
        m_claw = subsystem;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.ClawDown();
    }
    
    @Override
    public void end(boolean interrupted) {
        // m_claw.ClawStop();
    }

    @Override
    public boolean isFinished() {
        // m_claw.ClawStop();
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
