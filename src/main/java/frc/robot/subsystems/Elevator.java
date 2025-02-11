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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator implements Subsystem {
    TalonFX elevatorLead = new TalonFX(Constants.elevatorLeadID);
    TalonFX elevatorFollow = new TalonFX(Constants.elevatorFollowID);

    public TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

    public final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    public final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    public final NeutralOut m_brake = new NeutralOut();

    double elevatorSpeed = Constants.elevatorSpeed;

    public void ElevatorConfiguration() {
        elevatorConfigs.Slot0.kP = Constants.ELEVATORVOLTS_P_VALUE;
        elevatorConfigs.Slot0.kI = 0;
        elevatorConfigs.Slot0.kD = Constants.ELEVATORVOLTS_D_VALUE;

        elevatorConfigs.Voltage.withPeakForwardVoltage(Volts.of(Constants.peakVoltage))
            .withPeakReverseVoltage(Volts.of(-Constants.peakVoltage));
        
        elevatorConfigs.Slot1.kP = Constants.ELEVATORTORQUE_P_VALUE;
        elevatorConfigs.Slot1.kI = 0;
        elevatorConfigs.Slot1.kD = Constants.ELEVATORTORQUE_D_VALUE;

        elevatorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(Constants.peakAmps))
            .withPeakReverseTorqueCurrent(Amps.of(Constants.peakAmps));
        
        StatusCode statusLead = StatusCode.StatusCodeNotInitialized;
        StatusCode statusFollow = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusLead = elevatorLead.getConfigurator().apply(elevatorConfigs);
            statusFollow = elevatorFollow.getConfigurator().apply(elevatorConfigs);
            if (statusLead.isOK() && statusFollow.isOK()) break;
        }
        if (!statusLead.isOK() && !statusFollow.isOK()) {
            System.out.println("Could not apply configs to lead, error code: " + statusLead.toString());
            System.out.println("Could not apply configs to follow, error code: " + statusFollow.toString());
        }

        elevatorLead.setPosition(Constants.startPosition);
        elevatorFollow.setPosition(Constants.startPosition);

        elevatorFollow.setControl(new Follower(elevatorLead.getDeviceID(), true));

    }

    public Elevator() {
        ElevatorConfiguration();
    }

    public void ElevatorUp(){
        elevatorLead.set(Constants.elevatorSpeed);
    }

    public void ElevatorDown() {
        elevatorLead.set(-Constants.elevatorSpeed);
    }

    public void ElevatorStop() {
        elevatorLead.set(0);
    }
    
}
