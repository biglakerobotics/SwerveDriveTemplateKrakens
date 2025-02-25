package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ClawScore
 extends Command {
    private final Claw m_claw;

    public ClawScore
    (Claw claw) {
        m_claw = claw;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {

        m_claw.PickupPos();
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
