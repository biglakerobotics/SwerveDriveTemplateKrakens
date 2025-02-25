package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class RollerIntake implements Subsystem{ 
    
    TalonFX rollerMotor = new TalonFX(Constants.rollerID);
    public TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    public void rollerConfiguration(){
        rollerConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        StatusCode statusRoller = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusRoller = rollerMotor.getConfigurator().apply(rollerConfigs);
            if (statusRoller.isOK()) break;
        }
        if (!statusRoller.isOK()) {
            System.out.println("Could not apply configs to roller, error code: " + statusRoller.toString());
        }
    }
    
    

    public RollerIntake() {
        rollerConfiguration();
    }


    public void RollerIntake() {
        rollerMotor.set(Constants.rollerSpeed);
    }

    public void RollerStop() {
        rollerMotor.set(0);
    }

}
    


    

