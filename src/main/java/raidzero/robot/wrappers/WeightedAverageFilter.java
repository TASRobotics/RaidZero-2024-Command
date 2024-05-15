package raidzero.robot.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class that implements a weighted average filter for Pose2d objects.
 */
public class WeightedAverageFilter {
    private Translation2d sumTranslation = new Translation2d();
    private double sumRotation = 0.0;
    private double weightSum = 0.0;

    public WeightedAverageFilter() {
    }

    /**
     * Adds a Pose2d value to the filter with a specified weight.
     *
     * @param value  The Pose2d value to add.
     * @param weight The weight of the Pose2d value.
     */
    public void addValue(Pose2d value, double weight) {
        sumTranslation = sumTranslation.plus(value.getTranslation().times(weight));
        sumRotation += value.getRotation().getRadians() * weight;
        weightSum += weight;
    }

    /**
     * Returns the weighted average of all added Pose2d objects.
     *
     * @return The weighted average Pose2d.
     * @throws IllegalStateException If no values have been added to the filter.
     */
    public Pose2d getAverage() {
        if (weightSum != 0.0) {
            return new Pose2d(sumTranslation.div(weightSum), new Rotation2d(sumRotation / weightSum));
        } else {
            throw new IllegalStateException("No values have been added to the filter.");
        }
    }

    /**
     * Clears all values from the filter.
     */
    public void reset() {
        sumTranslation = new Translation2d();
        sumRotation = 0.0;
        weightSum = 0.0;
    }
}