package frc.raidzero.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.raidzero.Constants.VisionConstants;
import raidzero.wrappers.LimelightHelpers;
import raidzero.wrappers.LimelightHelpers.Results;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static final Swerve drive = Swerve.getInstance();

    private Alliance alliance;

    private Pose2d visionPose = new Pose2d();

    private MedianFilter noteXFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteYFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteAFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);

    private double noteX = 0;
    private double noteY = 0;
    private double noteA = 0;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
    }

    @Override
    public void onInit() {
        LimelightHelpers.getLatestResults(VisionConstants.APRILTAG_CAM_NAME);
    }

    @Override
    public void onStart(double timestamp) {
        alliance = DriverStation.getAlliance().get();
    }

    @Override
    public void update(double timestamp) {
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

    @Override
    public void stop() {
    }

    public void updatePose() {
        Results results = LimelightHelpers.getLatestResults(VisionConstants.APRILTAG_CAM_NAME).targetingResults;

        Pose2d robotPose = results.getBotPose2d_wpiBlue();
        double tl = results.latency_pipeline;
        double cl = results.latency_capture;

        if (robotPose.getX() != 0.0 && hasAprilTag()) {
            visionPose = robotPose;
            drive.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.XY_STDS, VisionConstants.XY_STDS, Units.degreesToRadians(VisionConstants.DEG_STDS)));
            drive.getPoseEstimator().addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
        }
    }

    public Pose2d getVisionPose(){
        return visionPose;
    }

    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;
        // if (!hasAprilTag()){
        //     return 0;
        // }
        double testReturn = drive.getPose().minus(speakerPose).getTranslation().getNorm();
        return drive.getPose().getTranslation().getDistance(speakerPose.getTranslation());
    }

    public Rotation2d getSpeakerAngle(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;
        // if (!hasAprilTag()){
        //     return null;
        // }
        Rotation2d testReturn = drive.getPose().minus(speakerPose).getTranslation().getAngle();
        return Rotation2d.fromRadians(Math.atan2(drive.getPose().getY() - speakerPose.getY(), drive.getPose().getX() - speakerPose.getX()));
    }

    public Boolean hasAprilTag(){
        return LimelightHelpers.getTV(VisionConstants.APRILTAG_CAM_NAME);
    }

    public void updateNote(){
        if (hasNote()) {
            noteX = noteXFilter.calculate(LimelightHelpers.getTX(VisionConstants.NOTE_CAM_NAME));
            noteY = noteYFilter.calculate(LimelightHelpers.getTY(VisionConstants.NOTE_CAM_NAME));
            noteA = noteAFilter.calculate(LimelightHelpers.getTA(VisionConstants.NOTE_CAM_NAME));
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
        return LimelightHelpers.getTV(VisionConstants.NOTE_CAM_NAME);
    }
}
