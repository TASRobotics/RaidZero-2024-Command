package raidzero.lib.math;

public class Conversions {
    /**
     * @param wheelRPS Wheel velocity in radians/second
     * @param circumference Wheel circumference in meters
     * @return Wheel velocity in meters/second
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        return wheelRPS * circumference;
    }

    /**
     * @param wheelMPS Wheel velocity in meters/second
     * @param circumference Wheel circumference in meters
     * @return Wheel velocity in rotations/second
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        return wheelMPS / circumference;
    }

    /**
     * @param wheelRotations Wheel position in rotations
     * @param circumference Wheel circumference in neters
     * @return Wheel distance in meters
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        return wheelRotations * circumference;
    }

    /**
     * @param wheelMeters Wheel distance in meters
     * @param circumference Wheel circumference in meters
     * @return Wheel position in rotations
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        return wheelMeters / circumference;
    }
}
