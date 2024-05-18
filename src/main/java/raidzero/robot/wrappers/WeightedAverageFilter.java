package raidzero.robot.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import raidzero.robot.wrappers.LimelightHelper.PoseEstimate;
import raidzero.robot.wrappers.LimelightHelper.RawFiducial;

public class WeightedAverageFilter {
    private RawFiducial[] rawFiducials = new RawFiducial[0];
    private PoseEstimate returnPoseEstimate = new PoseEstimate(
        new Pose2d(), 0.0, 0.0, 0,
         0.0, 0.0, 0.0, rawFiducials
    );
    private double weightSum = 0.0;
    private Translation2d sumTranslation = new Translation2d();
    private Rotation2d sumRotation = new Rotation2d();
    private double latestTimestamp = 0.0;
    private double avgTagDist = 0.0;

    public WeightedAverageFilter() {
        latestTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Adds a PoseEstimate value to the filter with a specified timestamp.
     *
     * @param poseEstimate The PoseEstimate value to add.
     */
    public void addValue(PoseEstimate poseEstimate) {
        Pose2d pose = poseEstimate.pose;
        double weight = calculateWeight(poseEstimate.timestampSeconds, poseEstimate.avgTagDist);
        returnPoseEstimate.avgTagDist += poseEstimate.avgTagDist/poseEstimate.tagCount;
        returnPoseEstimate.tagCount += poseEstimate.tagCount;
        returnPoseEstimate.avgTagArea += poseEstimate.avgTagArea/poseEstimate.tagCount;

        if (poseEstimate.timestampSeconds > latestTimestamp) {
            returnPoseEstimate.timestampSeconds = poseEstimate.timestampSeconds;
            returnPoseEstimate.latency = poseEstimate.latency;
            returnPoseEstimate.rawFiducials = poseEstimate.rawFiducials;
        }
        sumTranslation = sumTranslation.plus(pose.getTranslation().times(weight));
        sumRotation = pose.getRotation().plus(sumRotation.times(weight));
        weightSum += weight;
    }

    /**
     * Returns the weighted average of all added Pose2d objects.
     *
     * @return The weighted average Pose2d.
     * @throws IllegalStateException If no values have been added to the filter.
     */
    public PoseEstimate getAverage() {
        Pose2d sumPose = new Pose2d(sumTranslation, sumRotation);
        if (weightSum != 0.0) {
            returnPoseEstimate.pose = sumPose.div(weightSum);
            return returnPoseEstimate;
        } else {
            throw new IllegalStateException("No values have been added to the filter.");
        }
    }

    public double getWeightSum() {
        return weightSum;
    }

    /**
     * Clears all values from the filter.
     */
    public void reset() {
        sumTranslation = new Translation2d();
        sumRotation = new Rotation2d();
        weightSum = 0.0;
        latestTimestamp = 0.0;
        avgTagDist = 0.0;
        rawFiducials = new RawFiducial[0];
        returnPoseEstimate = new PoseEstimate(
            new Pose2d(), 0.0, 0.0, 0,
            0.0, 0.0, 0.0, rawFiducials
        );
    }
    
    /**
     * Calculates the weight based on the timestamp and avgTagDist.
     *
     * @param timestamp The timestamp of the PoseEstimate.
     * @return The weight.
     */
    private double calculateWeight(double timestamp, double avgTagDist) {
        double currentTime = Timer.getFPGATimestamp();
        double timeDifference = currentTime - timestamp;
        double weight = Math.exp(-Math.abs(timeDifference) - avgTagDist);
        weightSum += weight;
        return 0.0;
    }
}
