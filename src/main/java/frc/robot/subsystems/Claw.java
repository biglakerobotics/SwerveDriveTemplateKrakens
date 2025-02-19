package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

    public void ClawConfiguration() {
        clawConfigs.Slot0.kP = Constants.CLAWVOLTS_P_VALUE;
        clawConfigs.Slot0.kI = 0;
        clawConfigs.Slot0.kD = Constants.CLAWVOLTS_D_VALUE;

        clawConfigs.Voltage.withPeakForwardVoltage(Volts.of(Constants.peakVoltage))
            .withPeakReverseVoltage(Volts.of(-Constants.peakVoltage));
        
        clawConfigs.Slot1.kP = Constants.CLAWTORQUE_P_VALUE;
        clawConfigs.Slot1.kI = 0;
        clawConfigs.Slot1.kD = Constants.CLAWTORQUE_D_VALUE;

        clawConfigs.CurrentLimits.withStatorCurrentLimitEnable(true).withStatorCurrentLimit(Constants.peakAmps);
        clawConfigs.SoftwareLimitSwitch.withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(Constants.softForwardLimitClaw);
        clawConfigs.SoftwareLimitSwitch.withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(Constants.softReverseLimitClaw);

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
        clawLead.setNeutralMode(NeutralModeValue.Brake);

    }

    public Claw() {
        ClawConfiguration();
    }

    public void ClawUp(){
        // clawLead.set(Constants.clawSpeed);
        clawLead.set(xboxController.getRightY() * -Constants.clawSpeed);
    }

    public void ClawDown() {
        clawLead.set(xboxController.getRightY() * -Constants.clawSpeed);
    }
    

    public void ClawStop() {
        clawLead.set(0);
    }

    public void CoralLoadingPos(){
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawCoralLoadingPos));
    }
    public void ReefLevelOne() {
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawReefLevelOnePos));
    }

    public void ReefLevelTwo() {
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawReefLevelTwoPos));
    }

    public void ReefLevelThree() {
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawReefLevelThreePos));
    }

    public void ReefLevelFour() {
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawReefLevelFourPos));
    }

    public void TopOfclaw() {
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawTopOfElevator));
    }

    public void PickupPos() {
        clawLead.setControl(m_positionVoltage.withPosition(Constants.ClawPickupPos));
    }
    
}
