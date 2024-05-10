package raidzero.robot.subsystems;

import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
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
import raidzero.robot.wrappers.LimelightHelper.PoseEstimate;
import raidzero.robot.wrappers.LimelightHelper.Results;

public class Vision extends SubsystemBase implements Runnable{

    private static final Swerve swerve = Swerve.getInstance();

    private static Pigeon2 pigeon = swerve.getPigeon();

    private final BlockingQueue<Object> queue = new LinkedBlockingQueue<>();

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
            try {
                queue.take(); // Wait for a signal
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

    public PoseEstimate findBestPose(List<PoseEstimate> poses) {
    PoseEstimate bestPose = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (PoseEstimate pose : poses) {
        double distance = pose.avgTagDist
        long timeDifference = pose.; // Assuming PoseEstimate has a getTimestamp method

        // Calculate score based on distance and timestamp
        // This is just an example, you might want to adjust the formula to fit your needs
        double score = 1 / distance - timeDifference;

        if (score > bestScore) {
            bestScore = score;
            bestPose = pose;
        }
    }

    return bestPose;
}

    public void updatePose() {

        for(int llnum = 0; llnum< rawPoseEstimates.length; llnum++){
            LimelightHelper.SetRobotOrientation("limelight", pigeon.getAngle(), 0, 0, 0, 0, 0);
            PoseEstimate poseEstimate = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHTS[llnum]);
            rawPoseEstimates[llnum] = poseEstimate;
        }
        


        if (robotPose.getX() != 0.0 && hasAprilTag()) {
            visionPose = robotPose;
            swerve.getPoseEstimator().setVisionMeasurementStdDevs(
                VecBuilder.fill(
                    VisionConstants.XY_STDS,
                    VisionConstants.XY_STDS,
                    Conversions.degreesToRadians(VisionConstants.DEG_STDS)
                )
            );
            swerve.getPoseEstimator()
                .addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
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
