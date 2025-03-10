package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.commands.QuicklyClimbClimbCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber implements Subsystem{
    TalonFX climbTalonFX = new TalonFX(Constants.climbID);

    // XboxController xboxController = new XboxController(1);
    CommandXboxController xboxController = new CommandXboxController(0);

    public TalonFXConfiguration climbConfigs = new TalonFXConfiguration();

    public final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    public final NeutralOut m_brake = new NeutralOut();

    public void ClimbConfiguration() {
        climbConfigs.Slot0.kP = Constants.CLAWVOLTS_P_VALUE;
        climbConfigs.Slot0.kI = 0;
        climbConfigs.Slot0.kD = Constants.CLAWVOLTS_D_VALUE;

        climbConfigs.Voltage.withPeakForwardVoltage(Volts.of(Constants.peakVoltage))
            .withPeakReverseVoltage(Volts.of(-Constants.peakVoltage));
        
        climbConfigs.CurrentLimits.withStatorCurrentLimitEnable(true).withStatorCurrentLimit(Constants.peakAmps);
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = climbTalonFX.getConfigurator().apply(climbConfigs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs to lead, error code: " + status.toString());
        }

        climbTalonFX.setPosition(0);
        climbTalonFX.setNeutralMode(NeutralModeValue.Brake);
    }
    public Climber (){
        ClimbConfiguration();
    }
    public void QuicklyClimbClimb (){
        climbTalonFX.set(Constants.climbSpeed);
    }
    public void ClimberStop(){
        climbTalonFX.set(0);
    }
    
}
