// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autocommands.ClawScore;
import frc.robot.autocommands.CoralLoadingPos;
import frc.robot.autocommands.ElevatorStartPos;
import frc.robot.autocommands.PickupPos;
import frc.robot.autocommands.ReefLevelOne;
import frc.robot.autocommands.ReefLevelTwo;
import frc.robot.autocommands.ReefLevelThree;
import frc.robot.commands.AutoRollerIntakeCommand;
import frc.robot.commands.ClawDownCommand;
import frc.robot.commands.ClawTeleOp;
import frc.robot.commands.ClawUpCommand;
import frc.robot.commands.ElevatorTeleOp;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.commands.RollerIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.DRIVE_DEADBAND).withRotationalDeadband(MaxAngularRate * Constants.ANGULAR_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private SlewRateLimiter m_strafeX;
    private SlewRateLimiter m_strafeY;

    //Buttons
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController xboxController = new CommandXboxController(1);
    private final Joystick buttonBoard = new Joystick(2);
    private final JoystickButton pickUpPosButton = new JoystickButton(buttonBoard, 1);
    private final JoystickButton coralLoadingPosButton = new JoystickButton(buttonBoard, 2);
    private final JoystickButton reefLevelOnePosButton = new JoystickButton(buttonBoard,3);
    private final JoystickButton reefLevelTwoPosButton = new JoystickButton(buttonBoard, 4);
    private final JoystickButton reefLevelThreePosButton = new JoystickButton(buttonBoard, 5);
    private final JoystickButton rollerIntakeButton = new JoystickButton(buttonBoard, 6);

    //Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator m_elevator = new Elevator();
    public final Claw m_Claw = new Claw();
    public final RollerIntake m_RollerIntake = new RollerIntake();
    private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);

    //Commands
    private final CoralLoadingPos mCoralLoadingPos = new CoralLoadingPos(m_elevator, m_Claw);
    private final ReefLevelOne mReefLevelOne = new ReefLevelOne(m_elevator, m_Claw);
    private final ReefLevelTwo mReefLevelTwo = new ReefLevelTwo(m_elevator, m_Claw);
    private final ReefLevelThree mReefLevelThree = new ReefLevelThree(m_elevator, m_Claw);
    private final PickupPos mPickupPos = new PickupPos(m_elevator, m_Claw);
    private final RollerIntakeCommand mRollerIntakeCommand = new RollerIntakeCommand(m_RollerIntake);
    private final ClawScore mClawScore = new ClawScore(m_Claw);
    private final AutoRollerIntakeCommand mAutoRollerIntakeCommand = new AutoRollerIntakeCommand(m_RollerIntake);
    private final SendableChooser<Command> autoChooser = new SendableChooser();

    // public BooleanSupplier spinBoolean;

    // private final SendableChooser<Command> autoChooser;
    public RobotContainer() {
        // drivetrain.addVisionMeasurement(null, MaxAngularRate);
        
        // new PhotonVisionCommand(drivetrain).schedule();
       // private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);

       //Pathplanned command registration
       NamedCommands.registerCommand("LoadingPos", mCoralLoadingPos);
       NamedCommands.registerCommand("PickupPos", mPickupPos);
       NamedCommands.registerCommand("Level 1", mReefLevelOne);
       NamedCommands.registerCommand("Level 2", mReefLevelTwo);
       NamedCommands.registerCommand("Level 3", mReefLevelThree);
       NamedCommands.registerCommand("Intake", mAutoRollerIntakeCommand);
       NamedCommands.registerCommand("Score", mClawScore.withTimeout(.5));


       autoChooser.addOption("Test Auto",new PathPlannerAuto("J Start Crip"));
       autoChooser.addOption("Blood C&D", new PathPlannerAuto("Blood C&D"));

       autoChooser.setDefaultOption("Test Auto", new PathPlannerAuto("J Start Crip"));

       SmartDashboard.putData("AutoChooser", autoChooser);
       


       

        

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
        visionCommand.schedule();
        configureBindings();

        
        
        m_elevator.setDefaultCommand(new ElevatorTeleOp(m_elevator));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(m_strafeX.calculate(MathUtil.applyDeadband(-joystick.getLeftY(), Constants.DRIVE_DEADBAND)) * MaxSpeed)
            //     .withVelocityY(m_strafeY.calculate(MathUtil.applyDeadband(-joystick.getLeftX(), Constants.DRIVE_DEADBAND)) * MaxSpeed)
            //     .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), Constants.ANGULAR_DEADBAND) * MaxAngularRate)
        //)
        );
        
        m_strafeX = new SlewRateLimiter(Constants.SLEWRATELIMITER);
        m_strafeY = new SlewRateLimiter(Constants.SLEWRATELIMITER);

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //Elevator controller
        //Manual Elevator
        // bumper + joystick = move arm and elevator        
        xboxController.rightBumper().whileTrue(new ClawTeleOp(m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.leftBumper().whileTrue(new ElevatorTeleOp(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        //Presets
        // y reef 1
        // x reef 2
        // b reef 3
        // a reef 4
        // 6(share) pick up preset
        // 7(options) start pos
        // xboxController.button(7).whileTrue(new ElevatorStartPos(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // xboxController.button(6).whileTrue(new PickupPos(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.x().onTrue(new ReefLevelOne(m_elevator,m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.y().onTrue(new ReefLevelTwo(m_elevator,m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.b().onTrue(new ReefLevelThree(m_elevator,m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.a().onTrue(new CoralLoadingPos(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.back().onTrue(new PickupPos(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // xboxController.povDown().onTrue(new ClawScore(m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.povDown().onTrue(new ClawScore(m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.povUp().onTrue(new RollerIntakeCommand(m_RollerIntake).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

//      elevator presets for button board
        pickUpPosButton.onTrue(new PickupPos(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        coralLoadingPosButton.onTrue(new CoralLoadingPos(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        reefLevelOnePosButton.onTrue(new ReefLevelOne(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        reefLevelTwoPosButton.onTrue(new ReefLevelTwo(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        reefLevelThreePosButton.onTrue(new ReefLevelThree(m_elevator, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        rollerIntakeButton.whileTrue(new RollerIntakeCommand(m_RollerIntake).withInterruptBehavior(InterruptionBehavior.kCancelSelf));



        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return null;
        return autoChooser.getSelected();
    }
}
