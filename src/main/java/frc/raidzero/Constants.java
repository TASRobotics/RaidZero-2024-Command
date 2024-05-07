package frc.raidzero;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
}
