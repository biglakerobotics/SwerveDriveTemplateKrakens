package frc.robot.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ReefLevelTwo extends Command {
    private final Elevator m_elevator;
    private final Claw m_claw;
    private double goalTime = 0;
    private boolean check = false;

    public ReefLevelTwo(Elevator subsystem, Claw claw) {
        m_elevator = subsystem;
        m_claw = claw;
        addRequirements(m_elevator, m_claw);
    }

    @Override
    public void execute() {
        m_elevator.ReefLevelTwo();
        if (check != true) {
            check = true;
            System.out.println(check);
            goalTime = Timer.getTimestamp() + .5;
        }
        m_claw.ReefLevelTwo(goalTime);
    }

    @Override
    public void end(boolean interrupted) {
        check = false;
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
