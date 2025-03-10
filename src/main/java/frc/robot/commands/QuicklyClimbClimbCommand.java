package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class QuicklyClimbClimbCommand extends Command{
    private final Climber m_climber;
    public QuicklyClimbClimbCommand(Climber subsystem) {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        m_climber.QuicklyClimbClimb();
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.ClimberStop();
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

