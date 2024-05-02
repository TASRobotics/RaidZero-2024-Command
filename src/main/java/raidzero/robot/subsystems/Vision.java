package raidzero.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.lib.math.Conversions;
import raidzero.robot.wrappers.LimelightHelper;
import raidzero.robot.wrappers.LimelightHelper.Results;

public class Vision extends SubsystemBase {

    private static final Swerve swerve = Swerve.getSwerve();

    private Alliance alliance;

    private Pose2d visionPose = new Pose2d();

    private MedianFilter noteXFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteYFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteAFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);

    private double noteX = 0;
    private double noteY = 0;
    private double noteA = 0;

    public Vision() {
        alliance = DriverStation.getAlliance().get();

        LimelightHelper.getLatestResults(VisionConstants.APRILTAG_CAM_NAME);
    }

    public Pose2d getVisionPose(){
        return visionPose;
    }

    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance ==
            Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;
        // if (!hasAprilTag()){
        //     return 0;
        // }
        double testReturn = swerve.getPose().minus(speakerPose).getTranslation().getNorm();
        return swerve.getPose().getTranslation().getDistance(speakerPose.getTranslation());
    }

    public Rotation2d getSpeakerAngle(Alliance alliance) {
        Pose2d speakerPose = alliance ==
            Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;
        // if (!hasAprilTag()){
        //     return null;
        // }
        Rotation2d testReturn = swerve.getPose().minus(speakerPose).getTranslation().getAngle();
        return Rotation2d.fromRadians(
            Math.atan2(swerve.getPose().getY() - speakerPose.getY(),
            swerve.getPose().getX() - speakerPose.getX())
        );
    }

    public double getNoteX(){
        return noteX;
    }

    public double getNoteY(){
        return noteY;
    }

    public double getNoteA(){
        return noteA;
    }

    public Boolean hasNote(){
        return LimelightHelper.getTV(VisionConstants.NOTE_CAM_NAME);
    }

    public Boolean hasAprilTag(){
        return LimelightHelper.getTV(VisionConstants.APRILTAG_CAM_NAME);
    }

    public void updateNote(){
        if (hasNote()) {
            noteX = noteXFilter.calculate(LimelightHelper.getTX(VisionConstants.NOTE_CAM_NAME));
            noteY = noteYFilter.calculate(LimelightHelper.getTY(VisionConstants.NOTE_CAM_NAME));
            noteA = noteAFilter.calculate(LimelightHelper.getTA(VisionConstants.NOTE_CAM_NAME));
        }
        else {
            noteXFilter.reset();
            noteYFilter.reset();
            noteAFilter.reset();
            noteX = 0;
            noteY = 0;
            noteA = 0;
        }
    }

    public void updatePose() {
        Results results = 
            LimelightHelper.getLatestResults(VisionConstants.APRILTAG_CAM_NAME).targetingResults;

        Pose2d robotPose = results.getBotPose2d_wpiBlue();
        double tl = results.latency_pipeline;
        double cl = results.latency_capture;

        if (robotPose.getX() != 0.0 && hasAprilTag()) {
            visionPose = robotPose;
            //TODO: Implement getPoseEstimator() in Swerve.java
            // swerve.getPoseEstimator().setVisionMeasurementStdDevs(
            //     VecBuilder.fill(
            //         VisionConstants.XY_STDS,
            //         VisionConstants.XY_STDS,
            //         Conversions.degreesToRadians(VisionConstants.DEG_STDS)
            //     )
            // );
            // swerve.getPoseEstimator()
            //     .addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
        }
    }

    @Override
    public void periodic() {
        updatePose();
        updateNote();

        SmartDashboard.putNumber("Vision X", getVisionPose().getX());
        SmartDashboard.putNumber("Vision Y", getVisionPose().getY());
        SmartDashboard.putBoolean("Has April Tag", hasAprilTag());
        SmartDashboard.putNumber("Note X", getNoteX());
        SmartDashboard.putNumber("Note Y", getNoteY());
        SmartDashboard.putNumber("Note Area", getNoteA());
        SmartDashboard.putBoolean("Has Note", hasNote());
        SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance(alliance));

        try {
            SmartDashboard.putNumber("Speaker Angle", getSpeakerAngle(alliance).getDegrees());
        } catch (Exception e) {
        }
    }
    
}
