package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class TopOfElevator extends Command {
    private final Elevator m_elevator;

    public TopOfElevator(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.TopOfElevator();
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
