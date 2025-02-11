package frc.robot.commands;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PhotonVisionCommand extends Command {
    private final Vision vision;
    private final CommandSwerveDrivetrain drivetrain;

    public PhotonVisionCommand(CommandSwerveDrivetrain drivetrain) {
        this.vision = new Vision();
        this.drivetrain = drivetrain;

    }
    
    @Override
    public void execute(){

        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresent(est -> {

            var estStdDevs = vision.getEstimationStdDevs();

            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                Utils.fpgaToCurrentTime(est.timestampSeconds),
                estStdDevs);

        
        });
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}