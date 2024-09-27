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
import raidzero.robot.wrappers.LimelightHelpers;

public class Vision extends SubsystemBase {

    private Alliance alliance;

    private double noteX = 0;
    private double noteY = 0;
    private double noteA = 0;

    private MedianFilter noteAFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteXFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteYFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);

    private Pose2d visionPose = new Pose2d();
    
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.system();

    private static Vision vision;

    /**
     * Constructs a Vision object.
     */
    private Vision() {
        alliance = DriverStation.getAlliance().get();

        LimelightHelpers.getLatestResults(VisionConstants.APRILTAG_CAM_NAME);
    }

    //* Getters
    /**
     * Get the note area.
     * 
     * @return Note area
     */
    public double getNoteA(){
        return noteA;
    }

    /**
     * Get the note x position.
     * 
     * @return Note x position
     */
    public double getNoteX(){
        return noteX;
    }

    /**
     * Get the note y position.
     * 
     * @return Note y position
     */
    public double getNoteY(){
        return noteY;
    }

    /**
     * Get the vision pose.
     * 
     * @return Vision pose as {@link Pose2d}
     */
    public Pose2d getVisionPose(){
        return visionPose;
    }

    /**
     * Get the speaker angle.
     * 
     * @param alliance {@link Alliance} alliance
     * @return Speaker angle as {@link Rotation2d}
     */
    public Rotation2d getSpeakerAngle(Alliance alliance) {
        Pose2d speakerPose = alliance ==
            Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;

        return Rotation2d.fromRadians(
            Math.atan2(swerve.getPoseEstimator().getEstimatedPosition().getY()
             - speakerPose.getY(),
            swerve.getPoseEstimator().getEstimatedPosition().getX() - speakerPose.getX())
        );
    }

    
    /**
     * Get the speaker distance.
     * 
     * @param alliance {@link Alliance} alliance
     * @return Speaker distance
     */
    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance ==
            Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;

        return swerve.getPoseEstimator().getEstimatedPosition().getTranslation().getDistance(speakerPose.getTranslation());
    }


    //* Other methods


    /**
     * Checks if the vision detects an AprilTag.
     * 
     * @return True if the vision detects an AprilTag, false otherwise
     */
    public Boolean hasAprilTag(){
        return LimelightHelpers.getTV(VisionConstants.APRILTAG_CAM_NAME);
    }

    /**
     * Checks if the vision detects a note.
     * 
     * @return True if the vision detects a note, false otherwise
     */
    public Boolean hasNote(){
        return LimelightHelpers.getTV(VisionConstants.NOTE_CAM_NAME);
    }

     /**
     * Updates the note position.
     */
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

    /**
     * Updates the vision pose.
     */
    public void updatePose() {
        LimelightHelpers.LimelightResults results = 
            LimelightHelpers.getLatestResults(VisionConstants.APRILTAG_CAM_NAME);

        Pose2d robotPose = results.getBotPose2d_wpiBlue();
        double tl = results.latency_pipeline;
        double cl = results.latency_capture;

        if (robotPose.getX() != 0.0 && hasAprilTag()) {
            visionPose = robotPose;
            swerve.getPoseEstimator().setVisionMeasurementStdDevs(
                VecBuilder.fill(
                    VisionConstants.XY_STDS,
                    VisionConstants.XY_STDS,
                    Conversions.degreesToRadians(VisionConstants.DEG_STDS)
                )
            );
            swerve.addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
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
            System.out.println("could not get speaker angle");
        }
    }

    /**
     * Gets the vision subsystem.
     * 
     * @return {@link Vision} vision
     */
    public static Vision getSystem() {
        if (vision == null) {
            vision = new Vision();
        }
        return vision;
    }    
}
