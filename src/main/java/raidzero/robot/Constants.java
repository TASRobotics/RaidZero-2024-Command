package raidzero.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    
    public static final class Swerve {
        //* Device ID
        public static final int FL_THROTTLE_ID = 0;
        public static final int FR_THROTTLE_ID = 0;
        public static final int BL_THROTTLE_ID = 0;
        public static final int BR_THROTTLE_ID = 0;
        
        public static final int FL_AZIMUTH_ID = 0;
        public static final int FR_AZIMUTH_ID = 0;
        public static final int BL_AZIMUTH_ID = 0;
        public static final int BR_AZIMUTH_ID = 0;
        
        public static final int FL_ENCODER_ID = 0;
        public static final int FR_ENCODER_ID = 0;
        public static final int BL_ENCODER_ID= 0;
        public static final int BR_ENCODER_ID = 0;
        
        public static final int IMU_ID = 0;
        
        //* Offsets
        public static final double FL_AZIMUTH_OFFSET = -0.991943; // -0.809814 /*+ 0.5*/;
        public static final double FR_AZIMUTH_OFFSET = -0.482910 /*+ 0.5*/;
        public static final double BL_AZIMUTH_OFFSET = -0.880371 /*+ 0.5*/;
        public static final double BR_AZIMUTH_OFFSET = -0.955566 /*+ 0.5*/;
        
        //* Dimensions & conversions
        public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double DRIVE_BASE_RADIUS = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2.0;
        
        public static final double THROTTLE_GEAR_RATIO = (6.12 / 1.0);
        public static final double ROTOR_GEAR_RATIO = ((150.0 / 7.0) / 1.0);
        
        public static final double THROTTLE_VEL_CONVERSION_FACTOR =
        (1 / THROTTLE_GEAR_RATIO / 60) * WHEEL_DIAMETER_METERS * Math.PI;
        
        public static final double THROTTLE_POS_CONVERSTION_FACTOR =
        (1 / THROTTLE_GEAR_RATIO) * WHEEL_DIAMETER_METERS * Math.PI;
        
        // Swerve Kinematics
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );
            
        public static final double THROTTLE_ROT_TO_WHEEL_ROTATION = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double THROTTLE_WHEEL_ROT_TO_METERS = 1 / (Math.PI * WHEEL_DIAMETER_METERS);
        public static final double METERS_TO_THROTTLE_ROT = THROTTLE_ROT_TO_WHEEL_ROTATION / THROTTLE_WHEEL_ROT_TO_METERS;

        
        public static final double THROTTLE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double AZIMUTH_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
        
        public static final double MAX_VEL_MPS = 6000 * THROTTLE_REDUCTION / 60 * Math.PI * 0.102;// 4.959668;
        public static final double REAL_MAX_VEL_MPS = 4.2;
        public static final double MAX_ANGULAR_VEL_RPS = MAX_VEL_MPS * Math.sqrt(2) * TRACK_WIDTH;
        // public static final double TESTING_MAX_VEL_MPS = 3.0;
        // public static final double TESTING_MAX_ACCEL_MPSPS = 3.0;
        
        //* Azimuth PID values
        public static final double AZIMUTH_KP = 0.0;
        public static final double AZIMUTH_KI = 0.0;
        public static final double AZIMUTH_KD = 0.0;
        
        //* Throttle PID constants
        public static final double THROTTLE_KP = 0.0;
        public static final double THROTTLE_KI = 0.0;
        public static final double THROTTLE_KD = 0.0;
        public static final double THROTTLE_KF = 0.0;
        
        //* Throttle Characterization Values
        public static final double THROTTLE_KS = 0.0;
        public static final double THROTTLE_KV = 0.0;
        public static final double THROTTLE_KA = 0.0;
        
        //* Translation pathing PID constants
        public static final double TRANSLATION_KP = 0.0;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD = 0.0;
        
        //* Rotation pathing PID constants
        public static final double ROTATION_KP = 0.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        public static final double SNAP_CONTROLLER_KP = 0.1;
        public static final double SNAP_CONTROLLER_KI = 0.0;
        public static final double SNAP_CONTROLLER_KD = 0.0;
        public static final TrapezoidProfile.Constraints SNAP_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VEL_RPS, MAX_ANGULAR_VEL_RPS);
        public static final double kSnapControllerToleranceDegrees = 2.0;

        public static final double AIM_ASSIST_CONTROLLER_KP = 0.1;
        public static final double AIM_ASSIST_CONTROLLER_KI = 0.0;
        public static final double AIM_ASSIST_CONTROLLER_KD = 0.0;
        public static final TrapezoidProfile.Constraints AIM_ASSIST_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(REAL_MAX_VEL_MPS, REAL_MAX_VEL_MPS);


        public static final int THROTTLE_VEL_PID_SLOT = 0;
        public static final double THROTTLE_PID_UPDATE_HZ = 1000.0;
        public static final int AZIMUTH_POS_PID_SLOT = 0;
        public static final double AZIMUTH_PID_UPDATE_HZ = 1000.0;
        
        public static final InvertedValue THROTTLE_INVERSION = InvertedValue.Clockwise_Positive;
        public static final InvertedValue AZIMUTH_INVERSION = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue THROTTLE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue AZIMUTH_NEUTRAL_MODE = NeutralModeValue.Brake;
        
        public static final AbsoluteSensorRangeValue AZIMUTH_ENCODER_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue AZIMUTH_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
        
        public static CurrentLimitsConfigs AZIMUTH_CURRENT_LIMIT_CONFIGS = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(40)
            .withSupplyTimeThreshold(0.2);
        
        public static CurrentLimitsConfigs THROTTLE_CURRENT_LIMIT_CONFIGS = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentThreshold(60)
            .withSupplyTimeThreshold(0.2);

        public static final double TELEOP_RAMP_RATE = 0.25;
        
    }

    public static final class VisionConstants {
        public static final String APRILTAG_CAM_NAME = "limelight";
        public static final double XY_STDS = 0.1;
        public static final double DEG_STDS = 1;

        public static final String NOTE_CAM_NAME = "limelight-object";
        public static final int NOTE_FILTER_SIZE = 5;

        public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d());
    }
    
    public static final String CANBUS_ID = "seCANdary";

    public static final int CAN_TIMEOUT_MS = 10;
    public static final int LONG_CAN_TIMEOUT_MS = 100;
    
    public static final boolean ENABLE_FOC = true;

    public static final double STICK_DEADBAND = 0.02;

}
    