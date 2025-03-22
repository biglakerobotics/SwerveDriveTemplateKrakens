package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Claw implements Subsystem {
    TalonFX clawLead = new TalonFX(Constants.clawID);

    // XboxController xboxController = new XboxController(1);
    CommandXboxController xboxController = new CommandXboxController(1);

    public TalonFXConfiguration clawConfigs = new TalonFXConfiguration();

    public final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    public final MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);
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

        clawLead.setPosition(0);
        clawLead.setNeutralMode(NeutralModeValue.Brake);

        ClawMotionMagicConfigs();
    }

    public void ClawMotionMagicConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.2; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.03; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.05;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.CLAWCRUISEVELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.CLAWACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.CLAWJERK;

        clawLead.getConfigurator().apply(motionMagicConfigs);
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
        clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawCoralLoadingPos));
    }
    public void ReefLevelOne(double goalTime) {
        System.out.println(goalTime);
        System.out.println(Timer.getTimestamp());
        if (Timer.getTimestamp() >= goalTime) {
            clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawReefLevelOnePos));
        }
    }

    public void ReefLevelTwo(double goalTime) {
        if (Timer.getTimestamp() >= goalTime) {
            clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawReefLevelTwoPos));
        }
    }

    public void ReefLevelThree(double goalTime) {
        if (Timer.getTimestamp() >= goalTime) {
            clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawReefLevelOnePos));
        }
    }
    
    public void TopOfclaw() {
        clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawTopOfElevator));
    }

    public void PickupPos() {
        clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawPickupPos));
    }
    public void ClawScore(){
        clawLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ClawCoralLoadingPos));
    }
    
}
