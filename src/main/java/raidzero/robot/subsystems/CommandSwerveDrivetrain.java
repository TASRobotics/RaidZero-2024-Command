package raidzero.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.TunerConstants;
import raidzero.robot.wrappers.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private Field2d field = new Field2d();

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // private Vision vision = Vision.getSystem();

    private static CommandSwerveDrivetrain DriveTrain;

    private NeuralLimelight neuralLL = NeuralLimelight.getSystem();

    private boolean doRejectUpdateBack = false;
    private boolean doRejectUpdateLeft = false;
    private boolean doRejectUpdateRight = false;
    private boolean shouldHaveVision = true;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        // not sure if signlals should be refreshed or not... im sure it's fine
        positions[0] = Modules[0].getPosition(false);
        positions[1] = Modules[1].getPosition(false);
        positions[2] = Modules[2].getPosition(false);
        positions[3] = Modules[3].getPosition(false);

        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = Modules[0].getCurrentState();
        states[1] = Modules[1].getCurrentState();
        states[2] = Modules[2].getCurrentState();
        states[3] = Modules[3].getCurrentState();

        return states;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    SwerveDrivePoseEstimator getPoseEstimator() {
        return this.m_odometry;
    }

    public Field2d getField2d() {
        return this.field;
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("yaw", getState().Pose.getRotation().getDegrees());

        // this.addVisionMeasurement(
        //     LimelightHelper.getBotPose2d_wpiBlue("limelight-left"),
        //     Timer.getFPGATimestamp(),
        //     VecBuilder.fill(.5,.5,9999999)
        // );

        // this.addVisionMeasurement(
        //     LimelightHelper.getBotPose2d_wpiBlue("limelight-back"),
        //     Timer.getFPGATimestamp(),
        //     VecBuilder.fill(.5,.5,9999999)
        // );

        // this.m_odometry.addVisionMeasurement(
        //     LimelightHelper.getBotPose2d_wpiBlue("right"),
        //     Timer.getFPGATimestamp(),
        //     VecBuilder.fill(.5,.5,9999999)
        // );

        // LimelightHelper.Results res = LimelightHelper.getLatestResults("limelight-left").targetingResults;

        // LimelightHelpers.LimelightResults leftRes = 
        //     LimelightHelpers.getLatestResults("limelight-left");

        // LimelightHelpers.LimelightResults rightRes = 
        //     LimelightHelpers.getLatestResults("limelight-right");

        // LimelightHelpers.LimelightResults backRes = 
        //     LimelightHelpers.getLatestResults("limelight-back");

        // // if (Math.abs(this.getPigeon2().getRate()) > 720) {
        // //     doRejectUpdateBack = true;
        // //     doRejectUpdateLeft = true;
        // //     doRejectUpdateRight = true;
        // // }

        // // if (this.getPoseEstimator().getEstimatedPosition().getX() == 0.0) {
        // //     doRejectUpdateBack = false;
        // //     doRejectUpdateLeft = false;
        // //     doRejectUpdateRight = false;
        // // }

        // if (!LimelightHelpers.getTV("limelight-back")) {
        //     doRejectUpdateBack = true;
        // } else {
        //     doRejectUpdateBack = false;
        // }

        // if (!LimelightHelpers.getTV("limelight-left")) {
        //     doRejectUpdateLeft = true;
        // } else {
        //     doRejectUpdateLeft = false;
        // }

        // if (!LimelightHelpers.getTV("limelight-right")) {
        //     doRejectUpdateRight = true;
        // } else {
        //     doRejectUpdateRight = false;
        // }
        // if (!doRejectUpdateBack) {
        //     // this.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));

        //     this.addVisionMeasurement(
        //         // LimelightHelpers.getBotPose2d_wpiBlue("limelight-back"),
        //         backRes.getBotPose2d_wpiBlue(),
        //         (Timer.getFPGATimestamp() - (backRes.latency_pipeline / 1000.0) - (backRes.latency_capture / 1000.0)),
        //         VecBuilder.fill(.1,.1,9999999)
        //     );
        // }

        // if (!doRejectUpdateLeft) {
        //     // this.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));

        //     this.addVisionMeasurement(
        //         // LimelightHelpers.getBotPose2d_wpiBlue("limelight-left"),
        //         leftRes.getBotPose2d_wpiBlue(),
        //         (Timer.getFPGATimestamp() - (leftRes.latency_pipeline / 1000.0) - (leftRes.latency_capture / 1000.0)),
        //         VecBuilder.fill(.1,.1,9999999)
        //     );
        // }

        // if (!doRejectUpdateRight) {
        //     // this.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));

        //     this.addVisionMeasurement(
        //         // LimelightHelpers.getBotPose2d_wpiBlue("limelight-right"),
        //         rightRes.getBotPose2d_wpiBlue(),
        //         (Timer.getFPGATimestamp() - (rightRes.latency_pipeline / 1000.0) - (rightRes.latency_capture / 1000.0)),
        //         VecBuilder.fill(.1,.1,9999999)
        //     );
        // }

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

        if (mt2L.tagCount == 0) {
            doRejectUpdateLeft = true;
        } else {
            doRejectUpdateLeft = false;
        }

        if (shouldHaveVision && !doRejectUpdateLeft) {
            this.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));
            this.addVisionMeasurement(
                mt2L.pose,
                mt2L.timestampSeconds);
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");


        if (mt2R.tagCount == 0) {
            doRejectUpdateRight = true;
        } else {
            doRejectUpdateRight = false;
        }

        if (shouldHaveVision && !doRejectUpdateRight) {
            this.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));
            this.addVisionMeasurement(
                mt2R.pose,
                mt2R.timestampSeconds);
        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        if (mt2B.tagCount == 0) {
            doRejectUpdateBack = true;
        } else {
            doRejectUpdateBack = false;
        }

        if (shouldHaveVision && !doRejectUpdateBack) {
            this.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));
            this.addVisionMeasurement(
                mt2B.pose,
                mt2B.timestampSeconds);
        }

        field.setRobotPose(m_odometry.getEstimatedPosition());
    }

    public void stop() {
        this.setControl(AutoRequest.withSpeeds(
            new ChassisSpeeds(
                0.0,
                0.0,
                0.0
            )
        ));
    }

    public static CommandSwerveDrivetrain system() {
        if (DriveTrain == null) {
            DriveTrain = new CommandSwerveDrivetrain(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight);
        }
        return DriveTrain;
    }

    public ChassisSpeeds getRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose,
            this::seedFieldRelative,
            this::getRelativeSpeeds,
            (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(
                new PIDConstants(21.87, 0, 0), // 1.87
                new PIDConstants(2.3, 0, 0), //6.45
                3.5, // 3.5
                0.24,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        PPHolonomicDriveController.setRotationTargetOverride(neuralLL::getRotationalOverride);;
    }
}
