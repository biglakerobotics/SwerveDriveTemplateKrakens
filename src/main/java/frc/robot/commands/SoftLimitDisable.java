package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SoftLimitDisable extends Command {
    private final Elevator m_elevator;

    public SoftLimitDisable(Elevator subystem) {
        m_elevator = subystem;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.SoftLimitDisable();
    }

    // @Override
    // public void end(boolean interrupted) {
    //     m_elevator.RollerStop();
    // }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

}
