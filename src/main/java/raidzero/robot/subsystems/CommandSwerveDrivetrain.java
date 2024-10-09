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

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // private Vision vision = Vision.getSystem();

    private static CommandSwerveDrivetrain DriveTrain;

    private NeuralLimelight neuralLL = NeuralLimelight.getSystem();

    private boolean doRejectUpdateBack = false;
    private boolean doRejectUpdateLeft = false;
    private boolean doRejectUpdateRight = false;
    private boolean shouldHaveVision = true;
    private Pose2d leftPrev = null;
    private Pose2d rightPrev = null;
    private Pose2d backPrev = null;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-left",
            0.0,
            -0.26,
            0.19,
            0.0,
            46.0,
            90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right",
            0.0,
            0.26,
            0.19,
            0.0,
            46.0,
            -90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            -0.26,
            0.0,
            0.19,
            0.0,
            46.0,
            180.0
        );

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(90)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (mt2L != null) {
            leftPrev = mt2L.pose;
        } else {
            leftPrev = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().minus(Rotation2d.fromDegrees(90)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (mt2R != null) {
            rightPrev = mt2R.pose;
        } else {
            rightPrev = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (mt2B != null) {
            backPrev = mt2B.pose;
        } else {
            backPrev = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }

        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-left",
            0.0,
            -0.26,
            0.19,
            0.0,
            46.0,
            90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right",
            0.0,
            0.26,
            0.19,
            0.0,
            46.0,
            -90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            -0.26,
            0.0,
            0.19,
            0.0,
            46.0,
            180.0
        );

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(90)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (mt2L != null) {
            leftPrev = mt2L.pose;
        } else {
            leftPrev = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().minus(Rotation2d.fromDegrees(90)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (mt2R != null) {
            rightPrev = mt2R.pose;
        } else {
            rightPrev = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (mt2B != null) {
            backPrev = mt2B.pose;
        } else {
            backPrev = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
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

        if (this.getPigeon2().getRate() > 720) {
            shouldHaveVision = false;
        } else {
            shouldHaveVision = true;
        }

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(90)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        

        if (mt2L != null && mt2L.pose != null) {
            // SmartDashboard.putBoolean("Lpose", validPose(mt2L.pose));
            // SmartDashboard.putBoolean("Lpose", !(mt2L.tagCount == 0 || /*mt2L.tagCount > 1 ||*/ !validPose(mt2L.pose)  || (getLLposesDist(mt2L.pose, leftPrev) > 0.3)));
            SmartDashboard.putBoolean("Lcount", mt2L.tagCount == 0);
            SmartDashboard.putBoolean("Lvalid", !validPose(mt2L.pose));
            SmartDashboard.putBoolean("LDist", !(getLLposesDist(mt2L.pose, leftPrev) < 0.3));
            SmartDashboard.putNumber("LdistReal", getLLposesDist(mt2L.pose, leftPrev));
            if (mt2L.tagCount == 0 || /*mt2L.tagCount > 1 ||*/ !validPose(mt2L.pose)  || (getLLposesDist(mt2L.pose, leftPrev) > 0.3) || (mt2L.rawFiducials.length > 0 && mt2L.rawFiducials[0].ambiguity > 0.5)) {
                doRejectUpdateLeft = true;
            } else {
                doRejectUpdateLeft = false;
            }

            if (shouldHaveVision && !doRejectUpdateLeft) {
                SmartDashboard.putBoolean("Lpose", true);
                this.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));
                this.addVisionMeasurement(
                    // mt2L.pose,
                    new Pose2d(
                        mt2L.pose.getX(),
                        mt2L.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2L.timestampSeconds);
            } else {
                SmartDashboard.putBoolean("Lpose", false);
            }

            if (mt2L.rawFiducials.length > 0 && mt2L.rawFiducials[0] != null) {
                SmartDashboard.putNumber("leftamb", mt2L.rawFiducials[0].ambiguity);
            }
        }

        if (mt2L != null) {
            leftPrev = mt2L.pose;
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().minus(Rotation2d.fromDegrees(90)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (mt2R != null && mt2R.pose != null) {
            // SmartDashboard.putBoolean("Rpose", validPose(mt2R.pose));
            // SmartDashboard.putBoolean("Rpose", !(mt2R.tagCount == 0 || /*mt2R.tagCount > 1 ||*/ !validPose(mt2R.pose)  || (getLLposesDist(mt2R.pose, rightPrev) > 0.3)));
            if (mt2R.tagCount == 0 || /*mt2R.tagCount > 1 ||*/ !validPose(mt2R.pose) || (getLLposesDist(mt2R.pose, rightPrev) > 0.3) || (mt2R.rawFiducials.length > 0 && mt2R.rawFiducials[0].ambiguity > 0.5)) {
                doRejectUpdateRight = true;
            } else {
                doRejectUpdateRight = false;
            }

            if (shouldHaveVision && !doRejectUpdateRight) {
                SmartDashboard.putBoolean("Rpose",true);
                this.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));
                this.addVisionMeasurement(
                    // mt2R.pose,
                    new Pose2d(
                        mt2R.pose.getX(),
                        mt2R.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2R.timestampSeconds);
            } else {
                SmartDashboard.putBoolean("Rpose", false);
            }

            if (mt2R.rawFiducials.length > 0 && mt2R.rawFiducials[0] != null) {
                SmartDashboard.putNumber("rightamb", mt2R.rawFiducials[0].ambiguity);
            }

        }

        if (mt2R != null) {
            rightPrev = mt2R.pose;
        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), 0, 46.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        if (mt2B != null && mt2B.pose != null) {
            // SmartDashboard.putBoolean("Bpose", validPose(mt2B.pose));
            // SmartDashboard.putBoolean("Bpose", !(mt2B.tagCount == 0 || /*mt2B.tagCount > 1 ||*/ !validPose(mt2B.pose) || (getLLposesDist(mt2B.pose, backPrev) > 0.3)));
            if (mt2B.tagCount == 0 || /*mt2B.tagCount > 1 ||*/ !validPose(mt2B.pose) || (getLLposesDist(mt2B.pose, backPrev) > 0.3) || (mt2B.rawFiducials.length > 0 && mt2B.rawFiducials[0].ambiguity > 0.5)) {
                doRejectUpdateBack = true;
            } else {
                doRejectUpdateBack = false; 
            }

            if (shouldHaveVision && !doRejectUpdateBack) {
                SmartDashboard.putBoolean("Bpose",true);
                this.setVisionMeasurementStdDevs(VecBuilder.fill(.1,.1,9999999));
                this.addVisionMeasurement(
                    // mt2B.pose,
                    new Pose2d(
                        mt2B.pose.getX(),
                        mt2B.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2B.timestampSeconds
                );
            } else {
                SmartDashboard.putBoolean("Bpose", false);
            }

            if (mt2B.rawFiducials.length > 0 && mt2B.rawFiducials[0] != null) {
                SmartDashboard.putNumber("Backamb", mt2B.rawFiducials[0].ambiguity);
            }
        }

        if (mt2B != null) {
            backPrev = mt2B.pose;
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
