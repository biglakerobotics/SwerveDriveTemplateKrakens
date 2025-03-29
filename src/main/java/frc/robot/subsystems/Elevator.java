package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Elevator implements Subsystem {
    TalonFX elevatorLead = new TalonFX(Constants.elevatorLeadID);
    TalonFX elevatorFollow = new TalonFX(Constants.elevatorFollowID);
    public DigitalInput elevatorZeroSwitch = new DigitalInput(0);
    public boolean m_elevatorZeroTrue;
    public double elevatorPos;

    TalonFX clawMotor = new TalonFX(Constants.clawID);
    
    public final DynamicMotionMagicVoltage m_dynamicMotionMagicVoltage = 
    new DynamicMotionMagicVoltage(0, 80, 400, 4000);



    // XboxController xboxController = new XboxController(1);
    CommandXboxController xboxController = new CommandXboxController(1);

    public TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

    public final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    public final MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);  //Ian!!! i changed this to vvv
    public final MotionMagicVelocityVoltage m_MotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
    public final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    public final NeutralOut m_brake = new NeutralOut();

    double elevatorSpeed = Constants.elevatorSpeed;

    public void ElevatorConfiguration() {


        elevatorConfigs.Slot0.kP = Constants.ELEVATORVOLTS_P_VALUE;
        elevatorConfigs.Slot0.kI = Constants.ELEVATORVOLTS_I_VALUE;
        elevatorConfigs.Slot0.kD = Constants.ELEVATORVOLTS_D_VALUE;

        elevatorConfigs.Voltage.withPeakForwardVoltage(Volts.of(Constants.peakVoltage))
            .withPeakReverseVoltage(Volts.of(-Constants.peakVoltage));

        elevatorConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        elevatorConfigs.CurrentLimits.withStatorCurrentLimit(Constants.peakAmps).withStatorCurrentLimitEnable(true);
        elevatorConfigs.SoftwareLimitSwitch.withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(Constants.softForwardLimitElevator);
        elevatorConfigs.SoftwareLimitSwitch.withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(Constants.softReverseLimitElevator);

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
        
        ElevatorMotionMagicConfigs();

        elevatorFollow.setControl(new Follower(elevatorLead.getDeviceID(), true)); 

    }

    public void ElevatorMotionMagicConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.ELEVATOR_S_VALUE; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = Constants.ELEVATOR_V_VALUE; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = Constants.ELEVATOR_A_VALUE; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = Constants.ELEVATOR_P_VALUE; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = Constants.ELEVATOR_D_VALUE;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicExpo_kA = Constants.ELEVATORACCELERATION;
        // motionMagicConfigs.MotionMagicExpo_kV = Constants.ELEVATORCRUISEVELOCITY;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ELEVATORCRUISEVELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.ELEVATORACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.ELEVATORJERK;

        StatusCode statusLead = StatusCode.StatusCodeNotInitialized;
        StatusCode statusFollow = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusLead = elevatorLead.getConfigurator().apply(motionMagicConfigs);
            statusFollow = elevatorFollow.getConfigurator().apply(motionMagicConfigs);
            statusLead = elevatorLead.getConfigurator().apply(slot0Configs);
            statusFollow = elevatorFollow.getConfigurator().apply(slot0Configs);
            if (statusLead.isOK() && statusFollow.isOK()) {
                System.out.println("Configured Both Motors");
                break;
            }   
        }
        if (!statusLead.isOK() && !statusFollow.isOK()) {
            System.out.println("Could not apply configs to lead, error code: " + statusLead.toString());
            System.out.println("Could not apply configs to follow, error code: " + statusFollow.toString());
        }
    }

    public Elevator() {
        ElevatorConfiguration();
    }


    public boolean ElevatorStartPos() {
        if (!elevatorZeroSwitch.get()) {
            elevatorLead.set(-elevatorSpeed);
        } else {
            ElevatorStop();
            elevatorLead.setPosition(0);
            elevatorFollow.setPosition(0);
            m_elevatorZeroTrue = true;
            System.out.println("\n Hit LimitSwitch Setting Position To Zero");
        }
        return m_elevatorZeroTrue;
    }

    public void CoralLoadingPos() {
        elevatorLead.setControl(m_MotionMagicVoltage.withPosition(Constants.CoralLoadingPos));
        // elevatorLead.setControl(m_MotionMagicVelocityVoltage.withVelocity(Constants.CoralLoadingPos));
    }

    public void ReefLevelOne() {
        elevatorLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ReefLevelOnePos));
    }

    public void ReefLevelTwo() {
        elevatorLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ReefLevelTwoPos));
        // elevatorLead.setControl(m_MotionMagicVelocityVoltage.withVelocity(Constants.ReefLevelTwoPos));

        
    }

    public void ReefLevelThree() {
        elevatorLead.setControl(m_MotionMagicVoltage.withPosition(Constants.ReefLevelThreePos));
        // elevatorLead.setControl(m_MotionMagicVelocityVoltage.withVelocity(Constants.ReefLevelThreePos));
    }

    public void TopOfElevator() {
        elevatorLead.setControl(m_MotionMagicVoltage.withPosition(Constants.TopOfElevator));
        // elevatorLead.setControl(m_MotionMagicVelocityVoltage.withVelocity(Constants.TopOfElevator));

    }

    public void PickupPos() {
        elevatorLead.setControl(m_MotionMagicVoltage.withPosition(Constants.PickupPos));

        // elevatorLead.setControl(m_MotionMagicVelocityVoltage.withVelocity(Constants.PickupPos)); //Hi IAN!!!!! i changed the motion magic thing to motion magic voltage, maybe this will change things. You can change it back if it dont work 
        }

    public void ElevatorUp(){
        // elevatorLead.set(Constants.elevatorSpeed);
        elevatorLead.set(-xboxController.getLeftY()*Constants.elevatorSpeed);
    }

    public void ElevatorDown() {
        elevatorLead.set(-xboxController.getLeftY()*Constants.elevatorSpeed);
    }

    public void ElevatorStop() {
        elevatorLead.set(0);
    }
    public void SoftLimitDisable() {
        elevatorConfigs.SoftwareLimitSwitch.withForwardSoftLimitEnable(false);
        elevatorConfigs.SoftwareLimitSwitch.withReverseSoftLimitEnable(false);
        elevatorLead.getConfigurator().apply(elevatorConfigs);
        elevatorFollow.getConfigurator().apply(elevatorConfigs);
    }

    
}
