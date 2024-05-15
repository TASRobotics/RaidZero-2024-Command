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

    private final BlockingQueue<Object> queue = new LinkedBlockingQueue<>();

    private LinearFilter xFilter;
    private LinearFilter yFilter;
    private LinearFilter thetaFilter;


    private static Vision instance;
    private static Lock lock = new ReentrantLock();
    private static PoseEstimate bestPoseEstimate;
    private static PoseEstimate[] rawPoseEstimates;

    public static synchronized Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
            new Thread(instance).start();
        }
        return instance;
    }

    public static void setSharedVariable(int value) {
        lock.lock();
        try {
            sharedVariable = value;
        } finally {
            lock.unlock();
        }
    }

    public static int getSharedVariable() {
        lock.lock();
        try {
            return sharedVariable;
        } finally {
            lock.unlock();
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            double startTime = Timer.getFPGATimestamp();

            try {
                // Perform calculations
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

    private Pose2d visionPose = new Pose2d();

    private MedianFilter noteXFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteYFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);
    private MedianFilter noteAFilter = new MedianFilter(VisionConstants.NOTE_FILTER_SIZE);

    private double noteX = 0;
    private double noteY = 0;
    private double noteA = 0;

    private static Vision vision = new Vision();


    private Vision() {
        alliance = DriverStation.getAlliance().get();
        for (String limelight : VisionConstants.LIMELIGHTS) {
            LimelightHelper.getLatestResults(limelight);
        }
    }


    public Pose2d getVisionPose(){
        return visionPose;
    }

    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance ==
            Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;

        return swerve.getPose().getTranslation().getDistance(speakerPose.getTranslation());
    }

    public Rotation2d getSpeakerAngle(Alliance alliance) {
        Pose2d speakerPose = alliance ==
            Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;

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
        for (String limelight: VisionConstants.LIMELIGHTS){
            if (LimelightHelper.getTV(limelight)){
                return true;
            }
        }
        return false;
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
        if (rawPoseEstimates.length == 0)  return;
        // Sort the rawPoseEstimates by timestamp
        Arrays.sort(rawPoseEstimates, Comparator.comparingDouble(pose -> pose.timestampSeconds));
    
        // Initialize the filters with the first pose estimate
        PoseEstimate initialPose = rawPoseEstimates[0];
        xFilter = LinearFilter.singlePoleIIR(1.0, initialPose.pose.getX());
        yFilter = LinearFilter.singlePoleIIR(1.0, initialPose.pose.getY());
        thetaFilter = LinearFilter.singlePoleIIR(1.0, initialPose.pose.getRotation().getRadians());
    
        // Update the filters with each subsequent pose estimate
        for (int i = 1; i < rawPoseEstimates.length; i++) {
            PoseEstimate pose = rawPoseEstimates[i];
    
            // Calculate the gain based on the timestamp and the distance from the target
            double timeDifference = pose.timestampSeconds - initialPose.timestampSeconds;
            double distanceRatio = pose.avgTagDist/(pose.tagCount * initialPose.avgTagDist);
            double gain = Math.min(1.0, timeDifference * distanceRatio);
    
            // Update the filters
            xFilter.calculate(pose.getX() * gain);
            yFilter.calculate(pose.getY() * gain);
            thetaFilter.calculate(pose.getTheta() * gain);
        }
    }



    public void updatePose() {
        
        WeightedAverageFilter filteredPose;
        double latestTimestamp = Timer.getFPGATimestamp();
        double lastTimeStamp = 0;

        for (String limelight:VisionConstants.LIMELIGHTS) {
            LimelightHelper.SetRobotOrientation(limelight, pigeon.getAngle(), 0, 0, 0, 0, 0);
            PoseEstimate poseEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
            double weight = poseEstimate.tagCount / ( (latestTimestamp - poseEstimate.timestampSeconds) * poseEstimate.avgTagDist);
            filteredPose.addValue(poseEstimate.pose, weight);
            if (poseEstimate.timestampSeconds > lastTimeStamp) {
                lastTimeStamp = poseEstimate.timestampSeconds;
            }
        }

        if (filteredPose.getAverage().getX() != 0.0 && hasAprilTag()) {
            visionPose = filteredPose.getAverage();
            swerve.getPoseEstimator().setVisionMeasurementStdDevs(
                VecBuilder.fill(
                    VisionConstants.XY_STDS,
                    VisionConstants.XY_STDS,
                    Conversions.degreesToRadians(VisionConstants.DEG_STDS)
                )
            );
            swerve.getPoseEstimator().reset(fusedPose);
        }
    }



    @Override
    public void periodic() {
        queue.offer(new Object()); // Send a signal
    }

    public static Swerve getSwerve() {
        return swerve;
    }

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
