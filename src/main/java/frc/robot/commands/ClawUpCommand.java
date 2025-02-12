package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ClawUpCommand extends Command {
    private final Claw m_claw;
    public ClawUpCommand(Claw subsystem) {
        m_claw = subsystem;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.ClawUp();
    }

    @Override
    public void end(boolean interrupted) {
        m_claw.ClawStop();
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
