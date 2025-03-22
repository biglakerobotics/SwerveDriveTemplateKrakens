// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.module.Configuration;

import org.ejml.equation.Variable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PhotonVision.Camera;
import frc.robot.generated.TunerConstants;


public class LineUpSubsystem extends SubsystemBase {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /** Creates a new LineUpSubsystem. */
  PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.kFrontCameraName);
  CommandXboxController controller = new CommandXboxController(1);
  double forward = -controller.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  double strafe = -controller.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  double turn = -controller.getRightX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  boolean targetVisible = false;
  double targetYaw = 0.0;
  double targetRange = 0.0;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.DRIVE_DEADBAND).withRotationalDeadband(MaxAngularRate * Constants.ANGULAR_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use open-loop control for drive motors

  // PhotonPipelineResult pipelineResult = camera.getAllUnreadResults().get(0);
  // AprilTagDetection detection = pipelineResult.getBestTarget();


  public LineUpSubsystem() {

  }

  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();
    // This method will be called once per scheduler run
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
            targetYaw = target.getYaw();
            // NUMBERS DONT WORK GET THEM TO WORK THEY ARE SLACKING OFF AAAAAAAAAA
              targetRange = PhotonUtils.calculateDistanceToTargetMeters(0.5, 1.435, Units.degreesToRadians(-30), Units.degreesToRadians(target.getYaw()));
            targetVisible = true;
        }
      }
    }else{
      targetVisible = false;
    }
    
  }

  
  public void alignCommandReal(){
    if (controller.a().getAsBoolean() && targetVisible){
      turn = 
              // numbers are PID values. First one is angle. Second one is P
              (0.0 - targetYaw) * .1 * MaxAngularRate;
      
      forward = 
              (1.25 - targetRange) * .5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

      drivetrain.applyRequest(() -> drive.withVelocityX(-forward) // Drive forward with negative Y (forward)
           .withVelocityY(forward)  //idk which one is positive or negative, so good luck!!!!!
          .withRotationalRate(-turn) // Drive counterclockwise with negative X (left)
  );
      //gooner
    }
  }

  // drivetrain.drive(forward, strage, turn);
}
