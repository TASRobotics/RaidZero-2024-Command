package raidzero.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.system(); // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Swerve.STICK_DEADBAND)
            .withRotationalDeadband(MaxAngularRate * Constants.Swerve.STICK_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveWithAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * Constants.Swerve.STICK_DEADBAND)
            .withRotationalDeadband(MaxAngularRate * Constants.Swerve.STICK_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        configureBindings();

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

        SmartDashboard.putData(drivetrain.getField2d());
    }

    private void configureBindings() {
        driveWithAngle.HeadingController.setPID(Constants.Swerve.DIRECTION_SNAP_P, Constants.Swerve.DIRECTION_SNAP_I,
                Constants.Swerve.DIRECTION_SNAP_D);

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        // on right trigger make the rotation equal to the direction the stick is facing
        // this can be adapted to change to the angle of the game elemtents calculated
        // with vision in the future
        joystick.povUp()
                .whileTrue(drivetrain.applyRequest(() -> driveWithAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(
                                Rotation2d.fromDegrees(0))));

        joystick.povLeft()
                .whileTrue(drivetrain.applyRequest(() -> driveWithAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(
                                Rotation2d.fromDegrees(90))));

        joystick.povDown()
                .whileTrue(drivetrain.applyRequest(() -> driveWithAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(
                                Rotation2d.fromDegrees(180))));
        joystick.povRight()
                .whileTrue(drivetrain.applyRequest(() -> driveWithAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(
                                Rotation2d.fromDegrees(270))));

        // Brake on A button press
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the pigeon2 heading on right bumper press
        joystick.rightBumper().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0)));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
