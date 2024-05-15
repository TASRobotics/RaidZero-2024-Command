package raidzero.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;

public class Swerve extends SubsystemBase {
    
    private Pigeon2 pigeon;
    private SwerveModule moduleFL, moduleFR, moduleBL, moduleBR;

    private SwerveDrivePoseEstimator odometry;

    private Vision vision = Vision.getInstance();

    private Field2d field;

    private Alliance alliance;

    private PPHolonomicDriveController ppController;

    private PIDController mSnapController;

    private ProfiledPIDController mAimAssistYController;

    private static Swerve swerve = new Swerve();

    /**
     * Constructs a new Swerve object
     */
    private Swerve() {

        moduleFL = new SwerveModule(
            Constants.Swerve.FL_THROTTLE_ID,
            Constants.Swerve.FL_AZIMUTH_ID,
            Constants.Swerve.FL_ENCODER_ID,
            Constants.Swerve.FL_AZIMUTH_OFFSET
        );

        moduleFR = new SwerveModule(
            Constants.Swerve.FR_THROTTLE_ID,
            Constants.Swerve.FR_AZIMUTH_ID,
            Constants.Swerve.FR_ENCODER_ID,
            Constants.Swerve.FR_AZIMUTH_OFFSET
        );

        moduleBL = new SwerveModule(
            Constants.Swerve.BL_THROTTLE_ID,
            Constants.Swerve.BL_AZIMUTH_ID,
            Constants.Swerve.BL_ENCODER_ID,
            Constants.Swerve.BL_AZIMUTH_OFFSET
        );

        moduleBR = new SwerveModule(
            Constants.Swerve.BR_THROTTLE_ID,
            Constants.Swerve.BR_AZIMUTH_ID,
            Constants.Swerve.BR_ENCODER_ID,
            Constants.Swerve.BR_AZIMUTH_OFFSET
        );

        pigeon = new Pigeon2(Constants.Swerve.IMU_ID, Constants.CANBUS_ID);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.setYaw(0);

        odometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_DRIVE_KINEMATICS,
            getHeading(),
            getModulePositions(),
            getPose()
        );

        ppController = new PPHolonomicDriveController(
            new PIDConstants(
                Constants.Swerve.TRANSLATION_KP,
                Constants.Swerve.TRANSLATION_KI,
                Constants.Swerve.TRANSLATION_KD
            ),
            new PIDConstants(
                Constants.Swerve.ROTATION_KP,
                Constants.Swerve.ROTATION_KI,
                Constants.Swerve.ROTATION_KD
            ),
            Constants.Swerve.MAX_VEL_MPS,
            Constants.Swerve.DRIVE_BASE_RADIUS
        );

        mSnapController = new PIDController(
            Constants.Swerve.SNAP_CONTROLLER_KP, 
            Constants.Swerve.SNAP_CONTROLLER_KI, 
            Constants.Swerve.SNAP_CONTROLLER_KD
        );

        mAimAssistYController = new ProfiledPIDController(
            Constants.Swerve.AIM_ASSIST_CONTROLLER_KP, 
            Constants.Swerve.AIM_ASSIST_CONTROLLER_KI, 
            Constants.Swerve.AIM_ASSIST_CONTROLLER_KD, 
            Constants.Swerve.AIM_ASSIST_CONTROLLER_CONSTRAINTS
        );

        field = new Field2d();

        alliance = DriverStation.getAlliance().get();

        configureAutoBuilder();

    }

    //TODO: Reorganize methods

    /**
     * Drives the swerve drive with the given translation and rotation
     * 
     * @param translation Translation vector
     * @param rotation Rotation speed
     * @param fieldRelative If translation is field relative
     * @param isOpenLoop If swerve is open loop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation,
                                        getHeading())
            : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_VEL_MPS);

        moduleFL.setDesiredState(swerveModuleStates[0], isOpenLoop);
        moduleFR.setDesiredState(swerveModuleStates[1], isOpenLoop);
        moduleBL.setDesiredState(swerveModuleStates[2], isOpenLoop);
        moduleBR.setDesiredState(swerveModuleStates[3], isOpenLoop);
    }

    public synchronized void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // visionMeasurementStdDevs = new MatBuilder<N3, N1>(Nat.N3(),
            // Nat.N1()).fill(0.2, 0.2, 0.1);
            // mOdometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
            odometry.addVisionMeasurement(
                visionRobotPoseMeters,
                timestampSeconds,
                visionMeasurementStdDevs
            );
        } catch (Exception e) {
            System.out.println("Unable to add vision measurement to odometry.");
        }
    }

    public void enableTeleopRampRate(boolean enable) {
        ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();

        if (enable) {
            config.VoltageClosedLoopRampPeriod = Constants.Swerve.TELEOP_RAMP_RATE;
        } else {
            config.VoltageClosedLoopRampPeriod = 0;
        }

        moduleFL.getThrottle().getConfigurator().apply(config, Constants.CAN_TIMEOUT_MS);
        moduleFR.getThrottle().getConfigurator().apply(config, Constants.CAN_TIMEOUT_MS);
        moduleBL.getThrottle().getConfigurator().apply(config, Constants.CAN_TIMEOUT_MS);
        moduleBR.getThrottle().getConfigurator().apply(config, Constants.CAN_TIMEOUT_MS);
    }

    public void stop() {
        moduleFL.stopMotors();
        moduleFR.stopMotors();
        moduleBL.stopMotors();
        moduleBR.stopMotors();
    }

    /**
     * Sets the module states
     * 
     * @param desiredStates Desired {@link SwerveModuleState}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_VEL_MPS);

        moduleFL.setDesiredState(desiredStates[0], false);
        moduleFR.setDesiredState(desiredStates[1], false);
        moduleBL.setDesiredState(desiredStates[2], false);
        moduleBR.setDesiredState(desiredStates[3], false);
    }

    /**
     * Gets the module states
     * 
     * @return Array of {@link SwerveModuleState}
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = moduleFL.getState();
        states[1] = moduleFR.getState();
        states[2] = moduleBL.getState();
        states[3] = moduleBR.getState();

        return states;
    }

    /**
     * Gets the module positions
     * 
     * @return Array of {@link SwerveModulePosition}
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = moduleFL.getPosition();
        positions[1] = moduleFR.getPosition();
        positions[2] = moduleBL.getPosition();
        positions[3] = moduleBR.getPosition();

        return positions;
    }

    public SwerveModule[] getModules() {
        SwerveModule[] modules = new SwerveModule[4];

        modules[0] = moduleFL;
        modules[1] = moduleFR;
        modules[2] = moduleBL;
        modules[3] = moduleBR;

        return modules;
    }

    /**
     * Gets the current swerve pose
     * 
     * @return Current {@link Pose2d} pose
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Sets the current swerve pose
     * 
     * @param pose Desired {@link Pose2d} pose
     */
    public void setPose(Pose2d pose) {
        odometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
     * Gets the current heading of the swerve drive
     * 
     * @return Current {@link Rotation2d} heading
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getYawRate() {
        return pigeon.getRate();
    }

    public Field2d getField() {
        return field;
    }

    public PPHolonomicDriveController getPpController() {
        return ppController;
    }

    public PIDController getmSnapController() {
        return mSnapController;
    }

    public ProfiledPIDController getmAimAssistYController() {
        return mAimAssistYController;
    }

    /**
     * Sets the heading of the swerve drive
     * 
     * @param heading Desired heading
     */
    public void setHeading(double heading) {
        // odometry.resetPosition(getRotation(), getModulePositions(),
        //         new Pose2d(getPose().getTranslation(), heading));
        pigeon.setYaw(heading, Constants.CAN_TIMEOUT_MS);
    }

    /**
     * Zeros the heading of the swerve drive
     */
    public void zeroHeading() {
        odometry.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets the current rotation of the swerve drive
     * 
     * @return Current {@link Rotation2d} rotation
     */
    public Rotation2d getRotation() {
        return pigeon.getRotation2d();
    }

    /**
     * Resets the modules to absolute position
     */
    public void resetModulesToAbsolute() {
        moduleFL.resetToAbsolute();
        moduleBL.resetToAbsolute();
        moduleBR.resetToAbsolute();
        moduleFR.resetToAbsolute();
    }

    /**
     * Gets the relative speeds of the swerve drive
     * 
     * @return Relative {@link ChassisSpeeds} speeds
     */
    public ChassisSpeeds getRelativeSpeeds() {
        return Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the pigeon object
     * 
     * @return PigeonIMU
     */
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return odometry;
    }

    public Optional<Rotation2d> getRotationTargetOverride(){
        if (vision.hasNote()) {
            return Optional.of(Rotation2d.fromDegrees(vision.getNoteX()));
        } else {
            return Optional.empty();
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /**
     * Drives the swerve drive with the given speeds relative to swerve drive
     * 
     * @param speed Desired {@link ChassisSpeeds} speed
     */
    public void driveRelative(ChassisSpeeds speed) {
        setModuleStates(Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speed));
    }

    /**
     * Configures Pathplanner autobuilder
     */
    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRelativeSpeeds,
            this::driveRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    Constants.Swerve.TRANSLATION_KP,
                    Constants.Swerve.TRANSLATION_KI,
                    Constants.Swerve.TRANSLATION_KD
                ),
                new PIDConstants(
                    Constants.Swerve.ROTATION_KP,
                    Constants.Swerve.ROTATION_KI,
                    Constants.Swerve.ROTATION_KD
                ),
                Constants.Swerve.MAX_VEL_MPS,
                Constants.Swerve.TRACK_WIDTH / 2.0,
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();

                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            this
        );
    }

    /**
     * Gets the swerve object
     * 
     * @return Swerve object
     */
    public static Swerve getInstance() {
        return swerve;
    }

    @Override
    public void periodic() {
        odometry.update(getRotation(), getModulePositions());
        vision.getVisionPose().ifPresent(pose -> addVisionMeasurement(pose, Timer.getFPGATimestamp(), new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, 0.1)));
        field.setRobotPose(getPose());
    }

}
