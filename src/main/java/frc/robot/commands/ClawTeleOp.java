package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ClawTeleOp extends Command {
    private final Claw m_claw;

    public ClawTeleOp(Claw claw) {
        m_claw = claw;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.ClawUp();
    }
}
