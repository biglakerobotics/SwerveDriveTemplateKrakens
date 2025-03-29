package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
//  Sets the slew rate limit
    public static final int SLEWRATELIMITER = 3;
    public static final double DRIVE_DEADBAND = .1;
    public static final double ANGULAR_DEADBAND = .1;
//  Sets P and D values of the PIDs 
    public static final double DRIVE_P_VALUE = 0.12203; 
    public static final double DRIVE_D_VALUE = 0; 
    public static final double STEER_P_VALUE = 15; 
    public static final double STEER_D_VALUE = 0.02;
    public static final double ELEVATORVOLTS_P_VALUE = .8;
    public static final double ELEVATORVOLTS_I_VALUE = 0;
    public static final double ELEVATORVOLTS_D_VALUE = .065;
    public static final double CLAWVOLTS_P_VALUE = .24;
    public static final double CLAWVOLTS_D_VALUE = .0075;
    public static final double CLIMBVOLTS_P_VALUE = 0;
    public static final double CLIMBVOLTS_D_VALUE = 0;

// Motion Magic PID Configs

    public static final double ELEVATOR_P_VALUE = 0.8;
    public static final double ELEVATOR_D_VALUE = 0.065;

    public static final double ELEVATOR_S_VALUE = 0;
    public static final double ELEVATOR_V_VALUE = 0;
    public static final double ELEVATOR_A_VALUE = 0;

// Motion Magic Configs

    public static final double ELEVATORCRUISEVELOCITY = 175;
    public static final double ELEVATORACCELERATION = 175;
    public static final double ELEVATORJERK = 0;

    public static final double CLAWCRUISEVELOCITY = 75;
    public static final double CLAWACCELERATION = 75;
    public static final double CLAWJERK = 0;

//  Sets the KS value
    public static final double STEER_S_VALUE = 0.17318;
//  Elevator Constants
    public static final int elevatorLeadID = 4;
    public static final int elevatorFollowID = 5;
    
    public static final double elevatorSpeed = .9;
    public static final double peakVoltage = 9;
    public static final double peakAmps = 90;
    public static final double startPosition = 0;

    public static final double softForwardLimitElevator = 119;
    public static final double softReverseLimitElevator = 0;

//  Elevator Positons
    public static final double CoralLoadingPos = 29;
    public static final double ReefLevelOnePos = 18;
    public static final double ReefLevelTwoPos = 63;
    public static final double ReefLevelThreePos = 118;
    public static final double TopOfElevator = 118;
    public static final double PickupPos = 0;
// Claw Positions
    public static final double ClawCoralLoadingPos = -0.4;
    public static final double ClawReefLevelOnePos = 11;
    public static final double ClawReefLevelTwoPos = 11;
    public static final double ClawReefLevelThreePos = 11;
    public static final double ClawTopOfElevator = 0;
    public static final double ClawPickupPos = -0.4;
    
// Claw Constants
    public static final int clawID = 6;
    public static final double clawSpeed = .5;

    public static final double softForwardLimitClaw = 11.85;
    public static final double softReverseLimitClaw = -0.2;

//  Climber Constants
    public static final int climbID = 8;
    public static final double climbSpeed = -1;

//Roller Constants
public static final int rollerID = 7;
public static final double rollerSpeed = .05;

/// Vision Constants
    public static class VisionConstants {

        public static final String kFrontCameraName = "dumbdumbcamera";
        public static final String kBackCameraName = "BackCamera";

        public static final String kTopLeftCameraName ="TopRightCam";


        

        public static final Transform3d kRobotToTopRightCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(12.5), Units.inchesToMeters(10.5), Units.inchesToMeters(8.5) ),
            new Rotation3d(0,Units.degreesToRadians(-10),Units.degreesToRadians(-30)));
        
        public static final Transform3d kRobotToBackCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.5),Units.inchesToMeters(-12),Units.inchesToMeters(8.5)),
            new Rotation3d(0,Units.degreesToRadians(-15),Units.degreesToRadians(-150))
        );

        public static final Transform3d kRobotToTopLeftCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.5),Units.inchesToMeters(-12),Units.inchesToMeters(26)),
            new Rotation3d(0,Units.degreesToRadians(-15),Units.degreesToRadians(-150))
        );

     
        public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);
        
    public  static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4,4,8);
    public  static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1);

    public static final String[] CAMERA_NAMES = new String[] { kFrontCameraName , kBackCameraName, kTopLeftCameraName

    };

    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        kRobotToTopRightCam
        , kRobotToBackCam
        , kRobotToTopLeftCam

    };
}}

