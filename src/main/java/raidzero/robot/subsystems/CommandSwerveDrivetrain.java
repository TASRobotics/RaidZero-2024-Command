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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    public Field2d llfield = new Field2d();

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private static CommandSwerveDrivetrain DriveTrain;

    private NeuralLimelight neuralLL = NeuralLimelight.getSystem();

    private boolean ignoreFrontLime = false;
    private boolean ignoreRearLime = false;
    private boolean ignoreLeftLime = false;
    private boolean ignoreRightLime = false;
    private boolean ignoreAllLimes = true;

    private StructArrayPublisher<SwerveModuleState> modulePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private StructPublisher<Rotation3d> rotationPublisher = NetworkTableInstance.getDefault().getStructTopic("RotationState", Rotation3d.struct).publish();
    private StructPublisher <Pose2d> posefront = NetworkTableInstance.getDefault().getStructTopic("posefront", Pose2d.struct).publish();
    private StructPublisher <Pose2d> poseleft = NetworkTableInstance.getDefault().getStructTopic("poseleft", Pose2d.struct).publish();
    private StructPublisher <Pose2d> poseright = NetworkTableInstance.getDefault().getStructTopic("poseright", Pose2d.struct).publish();
    private StructPublisher <Pose2d> poseback = NetworkTableInstance.getDefault().getStructTopic("poseback", Pose2d.struct).publish();

    private LimelightHelpers.PoseEstimate limeFront, limeLeft, limeRight, limeBack;
    private LimelightHelpers.PoseEstimate limeFrontPrev, limeLeftPrev, limeRightPrev, limeBackPrev;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        this.initializeLimelightOdometry();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        this.initializeLimelightOdometry();
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

    public SwerveDrivePoseEstimator getPoseEstimator() {
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

        SmartDashboard.putNumber("Bot x", this.getPoseEstimator().getEstimatedPosition().getX());
        SmartDashboard.putNumber("Bot y", this.getPoseEstimator().getEstimatedPosition().getY());

        SmartDashboard.putNumber("Pigeon degrees", this.getPigeon2().getYaw().getValueAsDouble());

        modulePublisher.set(this.getModuleStates());
        rotationPublisher.set(this.getRotation3d());

        if (this.getPigeon2().getRate() > 720) {
            ignoreFrontLime = true;
            ignoreLeftLime = true;
            ignoreRightLime = true;
            ignoreRearLime = true;
        } else {
            ignoreFrontLime = false;
            ignoreLeftLime = false;
            ignoreRightLime = false;
            ignoreRearLime = false;
        }

        LimelightHelpers.SetRobotOrientation("limelight-front", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), this.getPigeon2().getRate(), 0.0, 0, 0, 0);
        limeFront = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

        if (limeFront != null && limeFront.pose != null) {
            ignoreFrontLime = //limeFront.tagCount == 0 ||
                            !validPose(limeFront.pose) ||
                            (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-front").getZ()) > 0.4) ||
                            (LimelightHelpers.getTA("limelight-front") < 0.1) ||
                            (limeFrontPrev != null && getLLposesDist(limeFront.pose, limeFrontPrev.pose) > 0.8) ||
                            (limeFrontPrev != null && (getLLposesDist(limeFront.pose, limeFrontPrev.pose) / (limeFront.timestampSeconds - limeFrontPrev.timestampSeconds)) > TunerConstants.kSpeedAt12VoltsMps) ||
                            (limeFront.rawFiducials.length > 0 && limeFront.rawFiducials[0].ambiguity > 0.5 && limeFront.rawFiducials[0].distToCamera > 3.5);

                            
            if (!ignoreAllLimes && !ignoreFrontLime) {
                SmartDashboard.putBoolean("Fpose", true);
                SmartDashboard.putString("FRONT", limeFront.pose.toString());
                posefront.set(limeFront.pose);

                this.addVisionMeasurement(
                    // new Pose2d(
                    //     limeFront.pose.getX(),
                    //     limeFront.pose.getY(),
                    //     this.getPigeon2().getRotation2d()
                    // ),
                    limeFront.pose,
                    limeFront.timestampSeconds,
                    VecBuilder.fill(0.7, 0.7,9999999).div(LimelightHelpers.getTA("limelight-front"))
                );
            } else {
                SmartDashboard.putBoolean("Fpose", false);
            }

            limeFrontPrev = limeFront;
        }

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), this.getPigeon2().getRate(), 0.0, 0, 0, 0);
        limeLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

        if (limeLeft != null && limeLeft.pose != null) {
            ignoreLeftLime = //limeLeft.tagCount == 0 ||
                            !validPose(limeLeft.pose) ||
                            (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-left").getZ()) > 0.4) ||
                            (LimelightHelpers.getTA("limelight-left") < 0.1) ||
                            (limeLeftPrev != null && getLLposesDist(limeLeft.pose, limeLeftPrev.pose) > 0.8) ||
                            (limeLeftPrev != null && (getLLposesDist(limeLeft.pose, limeLeftPrev.pose) / (limeLeft.timestampSeconds - limeLeftPrev.timestampSeconds)) > TunerConstants.kSpeedAt12VoltsMps) ||
                            (limeLeft.rawFiducials.length > 0 && limeLeft.rawFiducials[0].ambiguity > 0.5 && limeLeft.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreLeftLime) {
                SmartDashboard.putBoolean("Lpose", true);
                SmartDashboard.putString("LEFT", limeLeft.pose.toString());
                poseleft.set(limeLeft.pose);

                this.addVisionMeasurement(
                    // new Pose2d(
                    //     limeLeft.pose.getX(),
                    //     limeLeft.pose.getY(),
                    //     this.getPigeon2().getRotation2d()
                    // ),
                    limeLeft.pose,
                    limeLeft.timestampSeconds,
                    VecBuilder.fill(0.7, 0.7,9999999).div(LimelightHelpers.getTA("limelight-left"))
                );
            } else {
                SmartDashboard.putBoolean("Lpose", false);
            }

            limeLeftPrev = limeLeft;
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), this.getPigeon2().getRate(), 0.0, 0, 0, 0);
        limeRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (limeRight != null && limeRight.pose != null) {
            ignoreRightLime = //limeRight.tagCount == 0 ||
                            !validPose(limeRight.pose) ||
                            (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-right").getZ()) > 0.4) ||
                            (LimelightHelpers.getTA("limelight-right") < 0.1) ||
                            (limeRightPrev != null && getLLposesDist(limeRight.pose, limeRightPrev.pose) > 0.8) ||
                            (limeRightPrev != null && (getLLposesDist(limeRight.pose, limeRightPrev.pose) / (limeRight.timestampSeconds - limeRightPrev.timestampSeconds)) > TunerConstants.kSpeedAt12VoltsMps) ||
                            (limeRight.rawFiducials.length > 0 && limeRight.rawFiducials[0].ambiguity > 0.5 && limeRight.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreRightLime) {
                SmartDashboard.putBoolean("Rpose",true);
                SmartDashboard.putString("RIGHT", limeRight.pose.toString());
                poseright.set(limeRight.pose);

                this.addVisionMeasurement(
                    // new Pose2d(
                    //     limeRight.pose.getX(),
                    //     limeRight.pose.getY(),
                    //     this.getPigeon2().getRotation2d()
                    // ),
                    limeRight.pose,
                    limeRight.timestampSeconds,
                    VecBuilder.fill(0.7, 0.7,9999999).div(LimelightHelpers.getTA("limelight-right"))
                );
            } else {
                SmartDashboard.putBoolean("Rpose", false);
            }

            limeRightPrev = limeRight;
        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), this.getPigeon2().getRate(), 0.0, 0, 0, 0);
        limeBack = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        if (limeBack != null && limeBack.pose != null) {
            ignoreRearLime = //limeBack.tagCount == 0 ||
                            !validPose(limeBack.pose) ||
                            (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-back").getZ()) > 0.4) ||
                            (LimelightHelpers.getTA("limelight-back") < 0.1) ||
                            (limeBackPrev != null && getLLposesDist(limeBack.pose, limeBackPrev.pose) > 0.8) ||
                            (limeBackPrev != null && (getLLposesDist(limeBack.pose, limeBackPrev.pose) / (limeBack.timestampSeconds - limeBackPrev.timestampSeconds)) > TunerConstants.kSpeedAt12VoltsMps) ||
                            (limeBack.rawFiducials.length > 0 && limeBack.rawFiducials[0].ambiguity > 0.5 && limeBack.rawFiducials[0].distToCamera > 3.5);

            // SmartDashboard.putNumber("BackTA", LimelightHelpers.getTA("limelight-back"));
            // SmartDashboard.putNumber("backAmb", limeBack.rawFiducials[0].ambiguity);

            if (!ignoreAllLimes && !ignoreRearLime) {
                SmartDashboard.putBoolean("Bpose",true);
                SmartDashboard.putString("BACK", limeBack.pose.toString());
                poseback.set(limeBack.pose);

                this.addVisionMeasurement(
                    // new Pose2d(
                    //     limeBack.pose.getX(),
                    //     limeBack.pose.getY(),
                    //     this.getPigeon2().getRotation2d()
                    // ),
                    limeBack.pose,
                    limeBack.timestampSeconds,
                    VecBuilder.fill(0.7, 0.7,9999999).div(LimelightHelpers.getTA("limelight-back"))
                );
            } else {
                SmartDashboard.putBoolean("Bpose", false);
            }

            limeBackPrev = limeBack;
        }

        field.setRobotPose(m_odometry.getEstimatedPosition());
    }

    private double getLLposesDist(Pose2d curr, Pose2d prev) {
        return Math.sqrt(
            Math.pow((prev.getX() - curr.getX()), 2) +
            Math.pow((prev.getY() - curr.getY()), 2)
        );
    }

    private boolean validPose(Pose2d pose) {
        return pose.getX() > 0 && pose.getX() < 16 && pose.getY() > 0 && pose.getY() < 8;
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
                new PIDConstants(5, 0, 0),
                new PIDConstants(2.3, 0, 0),
                3.5,
                0.24,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        PPHolonomicDriveController.setRotationTargetOverride(neuralLL::getRotationalOverride);;
    }

    public void initializeLimelightOdometry() {
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-front",
            -0.2794,
            0.0,
            0.22,
            0.0,
            -10.0,
            180.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-left",
            -0.2032,
            0.2794,
            0.1577594,
            0.0,
            35.0,
            -90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right",
            -0.2032,
            -0.2794,
            0.1577594,
            0.0,
            35.0,
            90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            0.25485,
            0.0,
            0.19812,
            0.0,
            35.0,
            0.0
        );
    }
}
