package frc.robot.autocommands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ReefLevelOne extends Command {
    private final Elevator m_elevator;
    private final Claw m_claw;
    private double goalTime = 0;
    private boolean check = false;
    
    public ReefLevelOne(Elevator subsystem, Claw claw) {
            m_elevator = subsystem;
            m_claw = claw;
            addRequirements(m_elevator,m_claw);
    }

    @Override
    public void execute() {
        m_elevator.ReefLevelOne();
        if (check != true) {
            check = true;
            System.out.println(check);
            goalTime = Timer.getTimestamp() + 1;
        }
        m_claw.ReefLevelOne(goalTime);
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
