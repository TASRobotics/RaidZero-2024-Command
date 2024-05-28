package raidzero.robot.subsystems;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.lib.math.Conversions;
import raidzero.robot.wrappers.LimelightHelper;
import raidzero.robot.wrappers.WeightedAverageFilter;
import raidzero.robot.wrappers.LimelightHelper.PoseEstimate;
import raidzero.robot.wrappers.LimelightHelper.Results;

public class Vision extends SubsystemBase implements Runnable{

    private static final Swerve swerve = Swerve.getInstance();

    private static Pigeon2 pigeon = swerve.getPigeon();

    
    private static final PoseEstimate EMPTY_POSE_ESTIMATE = new PoseEstimate(new Pose2d(),
        0,0,0,0,0,0, null);
    private static PoseEstimate bestPoseEstimate = EMPTY_POSE_ESTIMATE;


    private static Vision instance;

    private static double bestPoseEstimateWeight;

    public static synchronized Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
            new Thread(instance).start();
        }
        return instance;
    }

    public static synchronized void setPoseEstimate(PoseEstimate poseEstimate) {
        bestPoseEstimate = poseEstimate;
    }
    
    public synchronized PoseEstimate getPoseEstimate() {
        return bestPoseEstimate;
    }

    public static synchronized void setPoseWeight(double weight) {
        bestPoseEstimateWeight = weight;
    }

    public synchronized double getPoseWeight() {
        return bestPoseEstimateWeight;
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            double startTime = Timer.getFPGATimestamp();

            try {
                // Perform calculations
                updatePose();
                updateNote();
    
                SmartDashboard.putNumber("Vision X", getPoseEstimate().pose.getX());
                SmartDashboard.putNumber("Vision Y", getPoseEstimate().pose.getY());
                SmartDashboard.putBoolean("Has April Tag", hasAprilTag());
                SmartDashboard.putNumber("Note X", getNoteXFilter().lastValue());
                SmartDashboard.putNumber("Note Y", getNoteYFilter().lastValue());
                SmartDashboard.putNumber("Note Area", getNoteAFilter().lastValue());
                SmartDashboard.putBoolean("Has Note", hasNote());
                SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance(alliance));
                SmartDashboard.putNumber("Speaker Angle", getSpeakerAngle(alliance).getDegrees());
    
                double endTime = Timer.getFPGATimestamp();
                double elapsedTime = endTime - startTime;
    
                if (elapsedTime > VisionConstants.LOOP_TIME) { // 0.01 seconds = 10 milliseconds
                    System.out.println("Loop overrun: " + (elapsedTime - 0.01) + " seconds");
                }
    
                // Sleep for the remaining time, if any
                double sleepTime = VisionConstants.LOOP_TIME - elapsedTime;
                if (sleepTime > 0) {
                    Thread.sleep((long)(sleepTime * 1000)); // Convert to milliseconds
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
        


        // Code to be executed in its own thread
        // For example, you can call setSharedVariable() here to update the shared variable


    private Alliance alliance;


    private MedianFilter noteXFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteYFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteAFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);

    private double noteX = 0;
    private double noteY = 0;
    private double noteA = 0;



    private Vision() {
        alliance = DriverStation.getAlliance().get();
        for (String limelight : VisionConstants.LIMELIGHTS) {
            LimelightHelper.getLatestResults(limelight);
        }
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
            Math.atan2(swerve.getPose().getY() - speakerPose.getY(),
            swerve.getPose().getX() - speakerPose.getX())
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

        return swerve.getPose().getTranslation().getDistance(speakerPose.getTranslation());
    }


    //* Other methods
    /**
     * Checks if the vision detects an AprilTag.
     * 
     * @return True if the vision detects an AprilTag, false otherwise
     */
    public Boolean hasAprilTag(){
        for (String limelight: VisionConstants.LIMELIGHTS){
            if (LimelightHelper.getTV(limelight)){
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if the vision detects a note.
     * 
     * @return True if the vision detects a note, false otherwise
     */
    public Boolean hasNote(){
        return LimelightHelper.getTV(VisionConstants.NOTE_CAM_NAME);
    }



    /**
     * Updates the note position.
     */
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




    /**
     * Updates the pose estimation using data from multiple Limelight cameras.
     * This method sets the robot orientation for each Limelight camera, retrieves the pose estimate,
     * calculates the weight for each pose estimate, and applies a weighted average filter to obtain
     * the filtered pose. If a valid filtered pose is obtained and the robot has an AprilTag,
     * the vision pose is updated and the pose estimator is reset.
     */
    public void updatePose() {
        
        WeightedAverageFilter filteredPose  = new WeightedAverageFilter(); // Initialize filteredPose variable;

        for (String limelight:VisionConstants.LIMELIGHTS) {
            LimelightHelper.SetRobotOrientation(limelight, pigeon.getAngle(), 0, 0, 0, 0, 0);
            PoseEstimate poseEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
            filteredPose.addValue(poseEstimate);
        }

        if (filteredPose.getWeightSum() > 0.0 && pigeon.getRate() < VisionConstants.MAX_ROTATION_RATE && hasAprilTag()) {
            setPoseEstimate(filteredPose.getAverage());
            setPoseWeight(filteredPose.getWeightSum());
        } else {
            setPoseEstimate(null);
            setPoseWeight(0.0);
        }
    }


    //* Setters

    public Alliance getAlliance() {
        return alliance;
    }

    public MedianFilter getNoteXFilter() {
        return noteXFilter;
    }

    public MedianFilter getNoteYFilter() {
        return noteYFilter;
    }

    public MedianFilter getNoteAFilter() {
        return noteAFilter;
    }
    
}
