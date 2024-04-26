package frc.raidzero.subsystems;

import edu.wpi.first.wpilibj.Timer;

public class Timestamp {
    private static Timestamp instance;
    private double startTime;

    private Timestamp() {
        startTime = Timer.getFPGATimestamp();
    }

    public static Timestamp getInstance() {
        if (instance == null) {
            instance = new Timestamp();
        }
        return instance;
    }

    public double getTimestamp() {
        double currentTime = Timer.getFPGATimestamp();
        return currentTime - startTime;
    }
}
