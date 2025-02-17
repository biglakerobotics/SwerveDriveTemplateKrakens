package frc.robot;

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
//  Sets P and D values of the PIDs 
    public static final double DRIVE_P_VALUE = 0.062203; 
    public static final double DRIVE_D_VALUE = 0; 
    public static final double STEER_P_VALUE = 10; 
    public static final double STEER_D_VALUE = 0.1; 
    public static final double ELEVATORVOLTS_P_VALUE = .5;
    public static final double ELEVATORVOLTS_D_VALUE = .1;
    public static final double ELEVATORTORQUE_P_VALUE = 0;
    public static final double ELEVATORTORQUE_D_VALUE = 0;
    public static final double CLAWVOLTS_P_VALUE = .4;
    public static final double CLAWVOLTS_D_VALUE = .01;
    public static final double CLAWTORQUE_P_VALUE = 0;
    public static final double CLAWTORQUE_D_VALUE = 0;
//  Sets the KS value
    public static final double STEER_S_VALUE = 0.16318;
//  Elevator Constants
    public static final int elevatorLeadID = 4;
    public static final int elevatorFollowID = 5;
    
    public static final double elevatorSpeed = .8;
    public static final double peakVoltage = 8;
    public static final double peakAmps = 70;
    public static final double startPosition = 0;

    public static final double softForwardLimitElevator = 107;
    public static final double softReverseLimitElevator = 0;

//  Elevator Positons
    public static final double CoralLoadingPos = 30;
    public static final double ReefLevelOnePos = 16;
    public static final double ReefLevelTwoPos = 50;
    public static final double ReefLevelThreePos = 107;
    public static final double TopOfElevator = 107;
    public static final double PickupPos = 0;
// Claw Positions
    public static final double ClawCoralLoadingPos = 0;
    public static final double ClawReefLevelOnePos = 10;
    public static final double ClawReefLevelTwoPos = 10;
    public static final double ClawReefLevelThreePos = 10;
    public static final double ClawReefLevelFourPos = 10;
    public static final double ClawTopOfElevator = 0;
    public static final double ClawPickupPos = 0;
    
//Claw Constants
public static final int clawID = 6;
public static final double clawSpeed = .2;

public static final double softForwardLimitClaw = 12;
public static final double softReverseLimitClaw = 0;

// Camera XYZ and Rotations
    //FrontCam
    public static final Double frontCameraX = 10.0;
    public static final Double frontCameraY = 10.0;
    public static final Double frontCameraZ = 8.5;

    public static final Double frontCameraRoll = 0.0;
    public static final Double frontCameraPitch = -10.0;
    public static final Double frontCameraYaw = 30.0;



    //BackCam
    public static final Double backCameraX = -10.0;
    public static final Double backCameraY = -10.0;
    public static final Double backCameraZ = 8.375;

    public static final Double backCameraRoll = 180.0;
    public static final Double backCameraPitch = 15.0;
    public static final Double backCameraYaw = -150.0;

/// Vision Constants
    public static class VisionConstants {
        public static final String kFrontCameraName = "dumbdumbcamera";
        public static final String kBackCameraName = "BackCamera";

        
        //front camera!
        public static final Transform3d kRobotToFrontCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(frontCameraX),Units.inchesToMeters(frontCameraY),Units.inchesToMeters(frontCameraZ)),
            new Rotation3d(frontCameraRoll,Units.degreesToRadians(frontCameraPitch),Units.degreesToRadians(frontCameraYaw))
        );

        //back camera!!
        public static final Transform3d kRobotToBackCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(backCameraX), Units.inchesToMeters(backCameraY), Units.inchesToMeters(backCameraZ) ),
            new Rotation3d(backCameraRoll,Units.degreesToRadians(backCameraPitch),Units.degreesToRadians(backCameraYaw)));
        
     
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public  static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public  static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    }
}

