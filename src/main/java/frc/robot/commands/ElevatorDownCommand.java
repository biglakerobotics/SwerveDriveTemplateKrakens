package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDownCommand extends Command {
    private final Elevator m_elevator;
    public ElevatorDownCommand(Elevator subsystem) {
        m_elevator = subsystem;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.ElevatorDown();
    }
    
    @Override
    public void end(boolean interrupted) {
        // m_elevator.ElevatorStop();
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
