// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.commands.GoToNote;
import raidzero.robot.subsystems.CommandSwerveDrivetrain;
import raidzero.robot.subsystems.NeuralLimelight;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 4.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.system(); // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> chooser;

	public RobotContainer() {    
        configureBindings();

        NeuralLimelight.getSystem().initialize();
        
        SmartDashboard.putData(drivetrain.getField2d());
        SmartDashboard.putData("llfield", drivetrain.llfield);


        NamedCommands.registerCommand("Go to note", new GoToNote().withTimeout(2.5));
        

        chooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("AutoChooser", chooser);
    }

	private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain
                .applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.rightBumper().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0)));

        // reset the field-centric heading on left bumper press
        //! Need to use `new Rotation2d()` since field heading is not reset in 2024 CTRE lib (Should be fixed in 2025)
        joystick.x().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldRelative(
                new Pose2d(
                    drivetrain.getPoseEstimator().getEstimatedPosition().getTranslation(),
                    new Rotation2d()
                )
            );
        }));

        joystick.y().onTrue(Commands.runOnce(() -> drivetrain.initializeLimelightOdometry()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        // drivetrain.registerTelemetry(logger::telemeterize);
    }

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
