// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClawTeleOp;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorTeleOp;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private SlewRateLimiter m_strafeX;
    private SlewRateLimiter m_strafeY;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController xboxController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator m_elevator = new Elevator();
    public final Claw m_Claw = new Claw();
    // public BooleanSupplier spinBoolean;

    private final SendableChooser<Command> autoChooser;
    public RobotContainer() {
        new PhotonVisionCommand(drivetrain).schedule();

        

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        
        m_elevator.setDefaultCommand(new ElevatorTeleOp(xboxController, m_elevator));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
drivetrain.applyRequest(() ->
                drive.withVelocityX(m_strafeX.calculate(MathUtil.applyDeadband(-joystick.getLeftY(), .1)) * MaxSpeed)
                .withVelocityY(m_strafeY.calculate(MathUtil.applyDeadband(-joystick.getLeftX(), .1)) * MaxSpeed)
                .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), .1) * MaxAngularRate)
        ));
        
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


        // xboxController.y().whileTrue(new ElevatorUpCommand(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // xboxController.a().whileTrue(new ElevatorDownCommand(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.leftBumper().whileTrue(new ElevatorTeleOp(xboxController, m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        xboxController.rightBumper().whileTrue(new ClawTeleOp(xboxController, m_Claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));


        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
