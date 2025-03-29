// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.module.Configuration;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PhotonVision.Camera;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAllignCommandCenter extends Command {

   private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final CommandSwerveDrivetrain m_drivetrain;
  // private final CommandXboxController m_joystick;
  public static final CommandXboxController m_joystick = new CommandXboxController(
    0);
  public static final XboxController m_joystickHID = m_joystick.getHID();

  PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.kFrontCameraName);
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
  .withDriveRequestType(DriveRequestType.Velocity);
  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 // Add a 10% deadband
    // withDriveRequestType(DriveRequestType.Velocity);
  
  /** Creates a new AutoAllignCommand. */
  public AutoAllignCommandCenter(CommandSwerveDrivetrain drivetrain) {

    // m_joystick = joystick;
    m_drivetrain = drivetrain;
    // m_joystickHID = joystickHID;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    

    if (hasTargets) { 
      PhotonTrackedTarget target = result.getBestTarget();
    double targetY = target.getBestCameraToTarget().getY();
    double targetX = target.getBestCameraToTarget().getX()-.7;
    double targetZ = target.getBestCameraToTarget().getZ();


    double p = .3;
    if (targetY>0.01 || targetY<-0.01) {
      System.out.println("Works!");
      m_drivetrain.setControl(drive.withVelocityY(targetY*MaxSpeed*p).withVelocityX(targetX*MaxSpeed*p).withRotationalRate(0));

    } else{
      m_drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

    }
      System.out.println("y "+targetY);
      System.out.println("x "+targetX);
      System.out.println("z "+targetZ);

    } else{
      System.out.println("no targets");
      m_drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

    }
    System.out.println("command called");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
