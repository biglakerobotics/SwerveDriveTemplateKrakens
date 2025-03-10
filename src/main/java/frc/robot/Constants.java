package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import java.util.List;
import java.util.stream.Stream;
import static java.util.stream.Collectors.toUnmodifiableList;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class Constants {
    

  /**
   * Constants for teleoperated driver control
   */
  public static class TeleopDriveConstants {
    /** Max velocity the driver can request */
    public static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts;
    /** Max angular velicity the driver can request */
    public static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.25);
  }
//  Sets the slew rate limit
    public static final int SLEWRATELIMITER = 3;
    public static final double DRIVE_DEADBAND = .1;
    public static final double ANGULAR_DEADBAND = .1;
//  Sets P and D values of the PIDs 
    public static final double DRIVE_P_VALUE = 0.12203; 
    public static final double DRIVE_D_VALUE = 0; 
    public static final double STEER_P_VALUE = 15; 
    public static final double STEER_D_VALUE = 0.02;
    public static final double ELEVATORVOLTS_P_VALUE = .475;
    public static final double ELEVATORVOLTS_I_VALUE = 0;
    public static final double ELEVATORVOLTS_D_VALUE = .075;
    public static final double ELEVATORTORQUE_P_VALUE = 0;
    public static final double ELEVATORTORQUE_I_VALUE = 0;
    public static final double ELEVATORTORQUE_D_VALUE = 0;
    public static final double CLAWVOLTS_P_VALUE = .24;
    public static final double CLAWVOLTS_D_VALUE = .0075;
    public static final double CLAWTORQUE_P_VALUE = 0;
    public static final double CLAWTORQUE_D_VALUE = 0;
//  Motion magic Configs for Elevator and Claw
    public static final MotionMagicConfigs ELEVATOR_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(40)
        .withMotionMagicAcceleration(120);

//  Sets the KS value
    public static final double STEER_S_VALUE = 0.17318;
//  Elevator Constants
    public static final int elevatorLeadID = 4;
    public static final int elevatorFollowID = 5;
    
    public static final double elevatorSpeed = .9;
    public static final double peakVoltage = 8;
    public static final double peakAmps = 70;
    public static final double startPosition = 29;

    public static final double softForwardLimitElevator = 111.5;
    public static final double softReverseLimitElevator = 0.08;

//  Elevator Positons
    public static final double CoralLoadingPos = 29;
    public static final double ReefLevelOnePos = 0.5;
    public static final double ReefLevelTwoPos = 47;
    public static final double ReefLevelThreePos = 111.5;
    public static final double TopOfElevator = 107;
    public static final double PickupPos = 0.1;
// Claw Positions
    public static final double ClawCoralLoadingPos = 0.01;
    public static final double ClawReefLevelOnePos = 10.15;
    public static final double ClawReefLevelTwoPos = 10.15;
    public static final double ClawReefLevelThreePos = 11;
    public static final double ClawTopOfElevator = 0;
    public static final double ClawPickupPos = 0.1;
    
//Claw Constants
    public static final int clawID = 6;
    public static final double clawSpeed = .25;

    public static final double softForwardLimitClaw = 11.85;
    public static final double softReverseLimitClaw = 0;

//Roller Constants
public static final int rollerID = 7;
public static final double rollerSpeed = .05;

/// Vision Constants
    public static class VisionConstants {
        public static final String kFrontCameraName = "dumbdumbcamera";
        public static final String kBackCameraName = "BackCamera";


        

        public static final Transform3d kRobotToFrontCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(12.5), Units.inchesToMeters(10.5), Units.inchesToMeters(8.5) ),
            new Rotation3d(0,Units.degreesToRadians(-10),Units.degreesToRadians(30)));
        
        public static final Transform3d kRobotToBackCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.5),Units.inchesToMeters(-12),Units.inchesToMeters(8.5)),
            new Rotation3d(0,Units.degreesToRadians(-15),Units.degreesToRadians(-150))
        );
     
        public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);
        
    public  static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4,4,8);
    public  static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1);

    public static final String[] CAMERA_NAMES = new String[] { kFrontCameraName , kBackCameraName
    };

    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        kRobotToFrontCam
        , kRobotToBackCam

    };
}

public static class DriveToPoseConstants {
    public static final LinearVelocity MAX_DRIVE_TO_POSE_TRANSLATION_VELOCITY = MAX_TELEOP_VELOCITY.div(2.0);
    public static final LinearAcceleration MAX_DRIVE_TO_POSE_TRANSLATION_ACCELERATION = MetersPerSecondPerSecond
        .of(2.0);
    public static final AngularVelocity MAX_DRIVE_TO_POSE_ANGULAR_VELOCITY = MAX_TELEOP_ANGULAR_VELOCITY.times(0.75);
    public static final AngularAcceleration MAX_DRIVE_TO_POSE_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond
        .of(6.0 * Math.PI);

    public static final double THETA_kP = 3.0;
    public static final double THETA_kI = 0.0;
    public static final double THETA_kD = 0.0;

    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;
}

  /** Constants for aligning to the reef */
  public static class AlignmentConstants {
    public static final int DEVICE_ID_RIGHT_CANRANGE = 38;
    public static final int DEVICE_ID_LEFT_CANRANGE = 39;

    public static final Distance ALIGNMENT_TOLERANCE = Inches.of(0.5);

    public static final LinearVelocity MAX_ALIGN_TRANSLATION_VELOCITY = MAX_TELEOP_VELOCITY.div(2.0);
    public static final LinearAcceleration MAX_ALIGN_TRANSLATION_ACCELERATION = MetersPerSecondPerSecond.of(6.0);
    public static final AngularVelocity MAX_ALIGN_ANGULAR_VELOCITY = MAX_TELEOP_ANGULAR_VELOCITY.times(0.75);
    public static final AngularAcceleration MAX_ALIGN_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond
        .of(6.0 * Math.PI);

    /** Pose of the robot relative to a reef branch for scoring coral on L4 */
    public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L4 = new Transform2d(
        inchesToMeters(-40),
        inchesToMeters(12),
        Rotation2d.fromDegrees(-90));
    /** Pose of the robot relative to a reef branch for scoring coral on L3 */
    public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L3 = new Transform2d(
        inchesToMeters(-40),
        inchesToMeters(12),
        Rotation2d.fromDegrees(-90));

    /** Pose of the robot relative to a reef branch for scoring coral on L2 */
    public static final Transform2d RELATIVE_SCORING_POSE_CORAL_L2 = new Transform2d(
        inchesToMeters(-40),
        inchesToMeters(12),
        Rotation2d.fromDegrees(90));

    // spotless:off
    /* The reef branches are in the arrays like this:
     *    ----------------------------------------
     *    |     5  / \ 6      |     11 / \ 0     |
     *    B    4 /     \ 7    |   10 /     \ 1   |
     *    L   3 |       | 8   |   9 |       | 2  R
     * +X U   2 |       | 9   |   8 |       | 3  E
     *    E    1 \     / 10   |    7 \     / 4   D
     *    |      0 \ / 11     |      6 \ / 5     |
     *    |___________________|__________________|
     * (0, 0)               +Y
     */
    // spotless:on
    /**
     * Poses of the right branches on the blue reef. Translation is the branch pipe base, rotation is pointing toward
     * reef center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_BLUE_RIGHT = Stream
        .of(
            new Pose2d(4.347746, 3.467, Rotation2d.fromDegrees(60)), // 0
              new Pose2d(3.942648, 3.840490, Rotation2d.fromDegrees(0)), // 2
              new Pose2d(4.062584, 4.398912, Rotation2d.fromDegrees(-60)), // 4
              new Pose2d(4.588763, 4.542161, Rotation2d.fromDegrees(-120)), // 6
              new Pose2d(4.98, 4.215, Rotation2d.fromDegrees(180)), // 8
              new Pose2d(4.873353, 3.632614, Rotation2d.fromDegrees(120))) // 10
        .collect(toUnmodifiableList());

    /**
     * Poses of the left branches on the blue reef. Translation is the branch pipe base, rotation is pointing toward
     * reef center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_BLUE_LEFT = Stream
        .of(
            new Pose2d(4.062584, 3.630770, Rotation2d.fromDegrees(60)), // 1
              new Pose2d(3.942648, 4.169106, Rotation2d.fromDegrees(0)), // 3
              new Pose2d(4.347175, 4.515, Rotation2d.fromDegrees(-60)), // 5
              new Pose2d(4.873926, 4.378820, Rotation2d.fromDegrees(-120)), // 7
              new Pose2d(4.994328, 3.841097, Rotation2d.fromDegrees(180)), // 9
              new Pose2d(4.589334, 3.466500, Rotation2d.fromDegrees(120)))// 11
        .collect(toUnmodifiableList());

    /**
     * Poses of the right branches on the red reef. Translation is the branch pipe base, rotation is pointing toward
     * reef center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_RED_RIGHT = Stream
        .of(
            new Pose2d(13.200254, 4.585000, Rotation2d.fromDegrees(-120)), // 0
              new Pose2d(13.605352, 4.211510, Rotation2d.fromDegrees(-180)), // 2
              new Pose2d(13.485416, 3.653088, Rotation2d.fromDegrees(120)), // 4
              new Pose2d(12.959237, 3.509839, Rotation2d.fromDegrees(60)), // 6
              new Pose2d(12.568000, 3.837000, Rotation2d.fromDegrees(0)), // 8
              new Pose2d(12.598000, 4.292000, Rotation2d.fromDegrees(-60))) // 10
        .collect(toUnmodifiableList());

    /**
     * Poses of the left branches on the red reef. Translation is the branch pipe base, rotation is pointing toward reef
     * center.
     */
    public static final List<Pose2d> REEF_BRANCH_POSES_RED_LEFT = Stream
        .of(
            new Pose2d(13.485416, 4.421230, Rotation2d.fromDegrees(-120)), // 1
              new Pose2d(13.605352, 3.882894, Rotation2d.fromDegrees(-180)), // 3
              new Pose2d(13.200825, 3.537000, Rotation2d.fromDegrees(120)), // 5
              new Pose2d(12.674074, 3.673180, Rotation2d.fromDegrees(60)), // 7
              new Pose2d(12.553672, 4.210903, Rotation2d.fromDegrees(0)), // 9
              new Pose2d(12.958666, 4.585500, Rotation2d.fromDegrees(-60)))// 11
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 left branches on the red alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 right branches on the red alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 left branches on the blue alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L4 right branches on the blue alliance */
    public static final List<Pose2d> REEF_L4_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L4))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 left branch on the red alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 right branch on the red alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 left branch on the blue alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L3 right branch on the blue alliance */
    public static final List<Pose2d> REEF_L3_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L3))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L2 left branch on the red alliance */
    public static final List<Pose2d> REEF_L2_SCORE_POSES_RED_LEFT = REEF_BRANCH_POSES_RED_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L2 right branch on the red alliance */
    public static final List<Pose2d> REEF_L2_SCORE_POSES_RED_RIGHT = REEF_BRANCH_POSES_RED_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L2 left branch on the blue alliance */
    public static final List<Pose2d> REEF_L2_SCORE_POSES_BLUE_LEFT = REEF_BRANCH_POSES_BLUE_LEFT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        .collect(toUnmodifiableList());

    /** Poses of the robot for scoring on L2 right on the blue alliance */
    public static final List<Pose2d> REEF_L2_SCORE_POSES_BLUE_RIGHT = REEF_BRANCH_POSES_BLUE_RIGHT.stream()
        .map(reefPose -> reefPose.plus(RELATIVE_SCORING_POSE_CORAL_L2))
        .collect(toUnmodifiableList());

    public static final Distance DISTANCE_TARGET_L4 = Meters.of(0.34);
    public static final Distance DISTANCE_TARGET_L3 = Meters.of(0.34);

    public static final Distance LATERAL_TARGET_L3_LEFT = Meters.of(0.05);
    public static final Distance LATERAL_TARGET_L3_RIGHT = Meters.of(0.02);

    public static final Distance LATERAL_TARGET_L4_LEFT = Meters.of(0.05);
    public static final Distance LATERAL_TARGET_L4_RIGHT = Meters.of(0.02);
  }
}

