package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorTeleOp extends Command {
    private final Elevator m_elevator;
    private final CommandXboxController m_xboxController;

    public ElevatorTeleOp(CommandXboxController xboxController, Elevator elevator) {
        m_elevator = elevator;
        m_xboxController = xboxController;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.ElevatorUp();
    }
}
