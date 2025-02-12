package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Claw;

public class ClawTeleOp extends Command {
    private final Claw m_claw;
    private final CommandXboxController m_xboxController;

    public ClawTeleOp(CommandXboxController xboxController, Claw claw) {
        m_claw = claw;
        m_xboxController = xboxController;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.ClawUp();
    }
}
