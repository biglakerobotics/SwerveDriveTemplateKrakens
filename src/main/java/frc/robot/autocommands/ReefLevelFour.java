package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ReefLevelFour extends Command {
    private final Elevator m_elevator;

    public ReefLevelFour(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.ReefLevelFour();
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
