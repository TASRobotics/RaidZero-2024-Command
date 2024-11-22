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
    private double llUpdateThreshold = 2.0;

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

    private boolean ignoreRearLime = false;
    private boolean ignoreLeftLime = false;
    private boolean ignoreRightLime = false;
    private boolean ignoreAllLimes = false;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.initializeLimelightOdometry();

        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.initializeLimelightOdometry();

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

        if (this.getPigeon2().getRate() > 720) {
            ignoreLeftLime = true;
            ignoreRightLime = true;
            ignoreRearLime = true;
        } else {
            ignoreLeftLime = false;
            ignoreRightLime = false;
            ignoreRearLime = false;
        }

        Pose2d currPose = this.getPoseEstimator().getEstimatedPosition();

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(90)).getDegrees(), this.getPigeon2().getRate(), 35.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        

        if (mt2L != null && mt2L.pose != null) {
            // SmartDashboard.putBoolean("Lpose", validPose(mt2L.pose));
            // SmartDashboard.putBoolean("Lpose", !(mt2L.tagCount == 0 || /*mt2L.tagCount > 1 ||*/ !validPose(mt2L.pose)  || (getLLposesDist(mt2L.pose, leftPrev) > 0.3)));
            SmartDashboard.putBoolean("Lcount", mt2L.tagCount == 0);
            SmartDashboard.putBoolean("Lvalid", !validPose(mt2L.pose));
            // SmartDashboard.putBoolean("LDist", !(getLLposesDist(mt2L.pose, leftPrev) < 0.3));
            // SmartDashboard.putNumber("LdistReal", getLLposesDist(mt2L.pose, leftPrev));
            ignoreLeftLime = mt2L.tagCount == 0 ||
                            !validPose(mt2L.pose) ||
                            (getLLposesDist(mt2L.pose, currPose) > llUpdateThreshold) ||
                            (mt2L.rawFiducials.length > 0 && mt2L.rawFiducials[0].ambiguity > 0.5 && mt2L.rawFiducials[0].distToCamera > 3.5);

            if (ignoreAllLimes && !ignoreLeftLime) {
                SmartDashboard.putBoolean("Lpose", true);

                this.addVisionMeasurement(
                    // mt2L.pose,
                    new Pose2d(
                        mt2L.pose.getX(),
                        mt2L.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2L.timestampSeconds,
                    VecBuilder.fill(.1,.1,9999999)
                );
            } else {
                SmartDashboard.putBoolean("Lpose", false);
            }

            llfield.setRobotPose(
                new Pose2d(
                    mt2L.pose.getX(),
                    mt2L.pose.getY(),
                    this.getPigeon2().getRotation2d()
                )
            );

            if (mt2L.rawFiducials.length > 0 && mt2L.rawFiducials[0] != null) {
                SmartDashboard.putNumber("leftamb", mt2L.rawFiducials[0].ambiguity);
                SmartDashboard.putNumber("LDistToTag", mt2L.rawFiducials[0].distToCamera);
            }
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().minus(Rotation2d.fromDegrees(90)).getDegrees(), this.getPigeon2().getRate(), 35.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (mt2R != null && mt2R.pose != null) {
            // SmartDashboard.putBoolean("Rpose", validPose(mt2R.pose));
            // SmartDashboard.putBoolean("Rpose", !(mt2R.tagCount == 0 || /*mt2R.tagCount > 1 ||*/ !validPose(mt2R.pose)  || (getLLposesDist(mt2R.pose, rightPrev) > 0.3)));
            ignoreRightLime = mt2R.tagCount == 0 ||
                            !validPose(mt2R.pose) ||
                            (getLLposesDist(mt2R.pose, currPose) > llUpdateThreshold) ||
                            (mt2R.rawFiducials.length > 0 && mt2R.rawFiducials[0].ambiguity > 0.5 && mt2R.rawFiducials[0].distToCamera > 3.5);

            if (ignoreAllLimes && !ignoreRightLime) {
                SmartDashboard.putBoolean("Rpose",true);

                this.addVisionMeasurement(
                    // mt2R.pose,
                    new Pose2d(
                        mt2R.pose.getX(),
                        mt2R.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2R.timestampSeconds,
                    VecBuilder.fill(.1,.1,9999999)
                );
            } else {
                SmartDashboard.putBoolean("Rpose", false);
            }

            llfield.setRobotPose(
                new Pose2d(
                    mt2R.pose.getX(),
                    mt2R.pose.getY(),
                    this.getPigeon2().getRotation2d()
                )
            );

            if (mt2R.rawFiducials.length > 0 && mt2R.rawFiducials[0] != null) {
                SmartDashboard.putNumber("rightamb", mt2R.rawFiducials[0].ambiguity);
                SmartDashboard.putNumber("RDistToCam", mt2R.rawFiducials[0].distToCamera);
            }

        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), this.getPigeon2().getRate(), 35.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        if (mt2B != null && mt2B.pose != null) {
            // SmartDashboard.putBoolean("Bpose", validPose(mt2B.pose));
            // SmartDashboard.putBoolean("Bpose", !(mt2B.tagCount == 0 || /*mt2B.tagCount > 1 ||*/ !validPose(mt2B.pose) || (getLLposesDist(mt2B.pose, backPrev) > 0.3)));
            ignoreRearLime = mt2B.tagCount == 0 ||
                            !validPose(mt2B.pose) ||
                            (getLLposesDist(mt2B.pose, currPose) > llUpdateThreshold) ||
                            (mt2B.rawFiducials.length > 0 && mt2B.rawFiducials[0].ambiguity > 0.5 && mt2B.rawFiducials[0].distToCamera > 3.5);

            if (ignoreAllLimes && !ignoreRearLime) {
                SmartDashboard.putBoolean("Bpose",true);

                this.addVisionMeasurement(
                    // mt2B.pose,
                    new Pose2d(
                        mt2B.pose.getX(),
                        mt2B.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2B.timestampSeconds,
                    VecBuilder.fill(.1,.1,9999999)
                );
            } else {
                SmartDashboard.putBoolean("Bpose", false);
            }

            llfield.setRobotPose(
                new Pose2d(
                    mt2B.pose.getX(),
                    mt2B.pose.getY(),
                    this.getPigeon2().getRotation2d()
                )
            );

            if (mt2B.rawFiducials.length > 0 && mt2B.rawFiducials[0] != null) {
                SmartDashboard.putNumber("Backamb", mt2B.rawFiducials[0].ambiguity);
                SmartDashboard.putNumber("BDistToTag", mt2B.rawFiducials[0].distToCamera);
            }
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
                new PIDConstants(21.87, 0, 0),
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
            "limelight-left",
            0.0,
            -0.254,
            0.15875,
            0.0,
            35.0,
            90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right",
            0.0,
            0.254,
            0.15875,
            0.0,
            35.0,
            -90.0
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-back",
            -0.254,
            0.0,
            0.18415,
            0.0,
            35.0,
            180.0
        );

        LimelightHelpers.SetRobotOrientation("limelight-left", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(90)).getDegrees(), this.getPigeon2().getRate(), 35.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2L = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        
        if (mt2L != null && mt2L.pose != null) {
            ignoreLeftLime = mt2L.tagCount == 0 ||
                            !validPose(mt2L.pose) ||
                            (mt2L.rawFiducials.length > 0 && mt2L.rawFiducials[0].ambiguity > 0.5 && mt2L.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreLeftLime) {
                SmartDashboard.putBoolean("Lpose", true);

                this.addVisionMeasurement(
                    // mt2L.pose,
                    new Pose2d(
                        mt2L.pose.getX(),
                        mt2L.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2L.timestampSeconds,
                    VecBuilder.fill(.1,.1,9999999)
                );
            }
        }

        LimelightHelpers.SetRobotOrientation("limelight-right", this.getPoseEstimator().getEstimatedPosition().getRotation().minus(Rotation2d.fromDegrees(90)).getDegrees(), this.getPigeon2().getRate(), 35.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2R = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (mt2R != null && mt2R.pose != null) {
            ignoreRightLime = mt2R.tagCount == 0 ||
                            !validPose(mt2R.pose) ||
                            (mt2R.rawFiducials.length > 0 && mt2R.rawFiducials[0].ambiguity > 0.5 && mt2R.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreRightLime) {
                SmartDashboard.putBoolean("Rpose",true);

                this.addVisionMeasurement(
                    // mt2R.pose,
                    new Pose2d(
                        mt2R.pose.getX(),
                        mt2R.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2R.timestampSeconds,
                    VecBuilder.fill(.1,.1,9999999)
                );
            }

        }

        LimelightHelpers.SetRobotOrientation("limelight-back", this.getPoseEstimator().getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), this.getPigeon2().getRate(), 35.0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2B = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        if (mt2B != null && mt2B.pose != null) {
            ignoreRearLime = mt2B.tagCount == 0 ||
                            !validPose(mt2B.pose) ||
                            (mt2B.rawFiducials.length > 0 && mt2B.rawFiducials[0].ambiguity > 0.5 && mt2B.rawFiducials[0].distToCamera > 3.5);

            if (!ignoreAllLimes && !ignoreRearLime) {
                SmartDashboard.putBoolean("Bpose",true);

                this.addVisionMeasurement(
                    // mt2B.pose,
                    new Pose2d(
                        mt2B.pose.getX(),
                        mt2B.pose.getY(),
                        this.getPigeon2().getRotation2d()
                    ),
                    mt2B.timestampSeconds,
                    VecBuilder.fill(.1,.1,9999999)
                );
            }
        }

        System.out.println("Done LL odom! at t: " + Timer.getFPGATimestamp());
    }
}
