package frc.raidzero;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;



public class Constants {

    // TODO: Implement a reader of field feature locations in JSON format that then imports into the constant file
    public static final class VisionConstants {
        public static final String APRILTAG_CAM_NAME = "limelight";
        public static final double XY_STDS = 0.1;
        public static final double DEG_STDS = 1;

        public static final String NOTE_CAM_NAME = "limelight-object";
        public static final int NOTE_FILTER_SIZE = 5;

        public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d());
    }

    public static final class LimelightConstants {
        public static final String kLimelightName = "limelight";
    }

    /**
     * Swerve Constants
     */
    public static final class SwerveConstants {
        /** Device IDs */
        public static final int kFrontLeftThrottleID = 1;
        public static final int kFrontRightThrottleID = 7;
        public static final int kRearLeftThrottleID = 3;
        public static final int kRearRightThrottleID = 5;

        public static final int kFrontLeftAzimuthID = 2;
        public static final int kFrontRightAzimuthID = 8;
        public static final int kRearLeftAzimuthID = 4;
        public static final int kRearRightAzimuthID = 6;

        public static final int kFrontLeftEncoderID = 1;
        public static final int kFrontRightEncoderID = 4;
        public static final int kRearLeftEncoderID = 2;
        public static final int kRearRightEncoderID = 3;

        public static final int kImuID = 0;

        /* Red Arm Offsets */
        // public static final double kFrontLeftAzimuthOffset = -0.463379;
        // public static final double kFrontRightAzimuthOffset = -0.565674 + 0.5;
        // public static final double kRearLeftAzimuthOffset = -0.090088;
        // public static final double kRearRightAzimuthOffset = -0.674316 + 0.5;

        /* Alpha Offsets */
        // public static final double kFrontLeftAzimuthOffset = 0.192383;
        // public static final double kFrontRightAzimuthOffset = 0.397949 + 0.5;
        // public static final double kRearLeftAzimuthOffset = 0.255615;
        // public static final double kRearRightAzimuthOffset = -0.205811 + 0.5;

        /* Sheesh Offsets */
        public static final double kFrontLeftAzimuthOffset = -0.991943; // -0.809814 /*+ 0.5*/;
        public static final double kFrontRightAzimuthOffset = -0.482910 /*+ 0.5*/;
        public static final double kRearLeftAzimuthOffset = -0.880371 /*+ 0.5*/;
        public static final double kRearRightAzimuthOffset = -0.955566 /*+ 0.5*/;

        public static final double kThrottleReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double kAzimuthReduction = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double kWheelDiameterMeters = 0.1016;

        public static final double kMaxVelMPS = 6000 * kThrottleReduction / 60 * Math.PI * 0.102;// 4.959668;
        public static final double kRealisticMaxVelMPS = 4.2;
        public static final double kTestingMaxVelMPS = 3.0;
        public static final double kTestingMaxAccelMPSPS = 3.0;

        // 20.75 OR 22.75 inches
        public static final double kTrackwidthMeters = Units.inchesToMeters(22.75);
        public static final double kWheelbaseMeters = Units.inchesToMeters(22.75);
        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0)
        );

        public static final double kMaxAngularVelRPS = kMaxVelMPS * Math.sqrt(2) * kTrackwidthMeters;


        public static final int kAzimuthPositionPIDSlot = 0;
        public static final double kAzimuth_kP = 50.0;// .75
        public static final double kAzimuth_kI = 0.0;
        public static final double kAzimuth_kD = 0.0;// 5.0
        public static final double kAzimuthPIDUpdateHz = 1000.0;

        public static final int kThrottleVelPIDSlot = 0;
        public static final double kThrottle_kP = 0.5;
        public static final double kThrottle_kI = 0.0;
        public static final double kThrottle_kD = 0.0;
        public static final double kThrottle_kV = 2.4;
        public static final double kThrottle_kA = 1.5;
        public static final double kThrottlePIDUpdateHz = 1000.0;

        public static final double kTranslationController_kP = 8.0; //5.0
        public static final double kTranslationController_kD = 0.0;
        public static final double kThetaController_kP = 2.5; //2.0
        public static final double kXControllerTolerance = 0.1; //0.1
        public static final double kYControllerTolerance = 0.1; //0.1
        public static final double kThetaControllerTolerance = Math.toRadians(5);

        public static final double kSnapController_kP = 0.1;
        public static final double kSnapController_kI = 0.0;
        public static final double kSnapController_kD = 0.0;
        public static final TrapezoidProfile.Constraints kSnapControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularVelRPS, kMaxAngularVelRPS);
        public static final double kSnapControllerToleranceDegrees = 2.0;

        public static final double kAimAssistController_kP = 0.1;
        public static final double kAimAssistController_kI = 0.0;
        public static final double kAimAssistController_kD = 0.0;
        public static final TrapezoidProfile.Constraints kAimAssistControllerConstraints =
            new TrapezoidProfile.Constraints(kRealisticMaxVelMPS, kRealisticMaxVelMPS);


        // Using SDS 6.75 ratio
        public static final double kThrottleRotToWheelRot = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double kThrottleWheelRotToMeters = 1 / (Math.PI * kWheelDiameterMeters);
        public static final double kMetersToThrottleRot = kThrottleRotToWheelRot/kThrottleWheelRotToMeters;

        public static final InvertedValue kThrottleInversion = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kAzimuthInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kThrottleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kAzimuthNeutralMode = NeutralModeValue.Brake;

        public static final AbsoluteSensorRangeValue kAzimuthEncoderRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue kAzimuthEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;

        public static CurrentLimitsConfigs kAzimuthCurrentLimitsConfigs = new CurrentLimitsConfigs();
        static {
            kAzimuthCurrentLimitsConfigs.SupplyCurrentLimit = 30;
            kAzimuthCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
            kAzimuthCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
            kAzimuthCurrentLimitsConfigs.SupplyTimeThreshold = 0.2;
        }

        public static CurrentLimitsConfigs kThrottleCurrentLimitsConfigs = new CurrentLimitsConfigs();
        static {
            kThrottleCurrentLimitsConfigs.SupplyCurrentLimit = 40;
            kThrottleCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
            kThrottleCurrentLimitsConfigs.SupplyCurrentThreshold = 60;
            kThrottleCurrentLimitsConfigs.SupplyTimeThreshold = 0.2;
        }

        public static final double kTeleopRampRate = 0.25;
    }

    
    public static final class DriveConstants {
        public static final Rotation2d STARTING_ROTATION = new Rotation2d(0.0);
        public static final Pose2d STARTING_POSE = new Pose2d(
                0.5,
                3.0,
                STARTING_ROTATION);
        // private static final double MAX_ACCEL_DISTANCE = 6.0 * Math.pow(TIMEOUT_S, 2);
        private static final double MAX_ACCEL_DISTANCE = 0.01;
        private static final double GYRO_ERROR_DEGREES_TIMEOUT = (0.4 / 60) * kCANTimeoutMs;
        public static final double CONFIDENCE_TO_ERROR = 1.0;
        public static final Matrix<N3, N1> STATE_STDEVS_MATRIX = MatBuilder.fill(
                Nat.N3(),
                Nat.N1(),new double[] {MAX_ACCEL_DISTANCE, MAX_ACCEL_DISTANCE, GYRO_ERROR_DEGREES_TIMEOUT});
        public static final Matrix<N3, N1> VISION_STDEVS_MATRIX = MatBuilder.fill(
                Nat.N3(),
                Nat.N1(),
                new double[] {1.0, 1.0, 1.0});
    }

    
    public static final int kCANTimeoutMs = 10;
    public static final int kLongCANTimeoutMs = 100; // constructors

    public static final String kCANBusName = "seCANdary";

    public static final double kMaxMotorVoltage = 12.0;

    public static final boolean kEnableFOC = true;

    public static final double kJoystickDeadband = 0.1;


}
