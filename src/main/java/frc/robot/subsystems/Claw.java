package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Claw implements Subsystem {
    TalonFX clawLead = new TalonFX(Constants.clawID);

    // XboxController xboxController = new XboxController(1);
    CommandXboxController xboxController = new CommandXboxController(1);

    public TalonFXConfiguration clawConfigs = new TalonFXConfiguration();

    public final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    public final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    public final NeutralOut m_brake = new NeutralOut();

    double clawSpeed = Constants.clawSpeed;

    public void ClawConfiguration() {
        clawConfigs.Slot0.kP = Constants.CLAWVOLTS_P_VALUE;
        clawConfigs.Slot0.kI = 0;
        clawConfigs.Slot0.kD = Constants.CLAWVOLTS_D_VALUE;

        clawConfigs.Voltage.withPeakForwardVoltage(Volts.of(Constants.peakVoltage))
            .withPeakReverseVoltage(Volts.of(-Constants.peakVoltage));
        
        clawConfigs.Slot1.kP = Constants.CLAWTORQUE_P_VALUE;
        clawConfigs.Slot1.kI = 0;
        clawConfigs.Slot1.kD = Constants.CLAWTORQUE_D_VALUE;

        clawConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(Constants.peakAmps))
            .withPeakReverseTorqueCurrent(Amps.of(Constants.peakAmps));
        
        StatusCode statusLead = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusLead = clawLead.getConfigurator().apply(clawConfigs);
            if (statusLead.isOK()) break;
        }
        if (!statusLead.isOK()) {
            System.out.println("Could not apply configs to lead, error code: " + statusLead.toString());
        }

        clawLead.setPosition(Constants.startPosition);

    }

    public Claw() {
        ClawConfiguration();
    }

    public void ClawUp(){
        // clawLead.set(Constants.clawSpeed);
        clawLead.set(xboxController.getRightY()*.1);
    }

    public void ClawDown() {
        clawLead.set(xboxController.getRightY()*.1);
    }

    public void ClawStop() {
        clawLead.set(0);
    }
    
}
