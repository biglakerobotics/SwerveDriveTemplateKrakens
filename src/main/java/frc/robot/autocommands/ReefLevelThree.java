package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ReefLevelThree extends Command {
    private final Elevator m_elevator;
    private final Claw m_claw;

    public ReefLevelThree(Elevator subsystem, Claw claw) {
        m_elevator = subsystem;
        m_claw = claw;
        addRequirements(m_elevator, m_claw);
    }

    @Override
    public void execute() {
        m_elevator.ReefLevelThree();
        m_claw.ReefLevelThree();
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
