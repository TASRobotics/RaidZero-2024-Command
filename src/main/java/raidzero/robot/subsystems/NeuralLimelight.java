package raidzero.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.LimelightHelpers;

public class NeuralLimelight extends SubsystemBase {

    private final String limelightName = Constants.VisionConstants.NOTE_CAM_NAME;
    private static final NeuralLimelight neuralLimelight = new NeuralLimelight();

    private NeuralLimelight() {
        LimelightHelpers.setLEDMode_ForceBlink(limelightName);
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }

    public void initialize() {
        SmartDashboard.putNumber("Note TX", LimelightHelpers.getTX(limelightName));
        SmartDashboard.putNumber("Note TY", LimelightHelpers.getTY(limelightName));
        SmartDashboard.putNumber("Note TA", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putBoolean("Note detected", LimelightHelpers.getTV(limelightName));
    }

    /**
     * Gets the horizontal offset from target note as a {@link Rotation2d}.
     * 
     * @return The horizontal offset from target note as a {@link Rotation2d}.
     */
    public Rotation2d getNoteRotation() {
        return Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName));
    }

    /**
     * Gets the field relative horizontal offset from target note as a {@link Rotation2d}.
     * 
     * @return The field relative horizontal offset from target note as a {@link Rotation2d}.
     */
    public Rotation2d getNoteFieldRotation() {
        return CommandSwerveDrivetrain.system().getPigeon2().getRotation2d().minus(this.getNoteRotation());
    }

    /**
     * If a note is detected.
     * 
     * @return If a note is detected.
     */
    public boolean isNoteDetected() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets the area of the note. (Relative to camera FOV...?)
     * 
     * @return
     */
    public double getNoteArea() {
        return LimelightHelpers.getTA(limelightName);
    }

    // public double getNoteDistance() {
    //     return Constants.VisionConstants.NOTE_HEIGHT / Math.tan(Math.toRadians(Constants.VisionConstants.NOTE_MOUNTING_ANGLE + LimelightHelper.getTY(limelightName)));
    // }

    /**
     * Gets the horizontal offset from target note.
     * 
     * @return The horizontal offset from target note.
     */
    public double getNoteTx() {
        return LimelightHelpers.getTX(limelightName);
    }

    /**
     * Gets the vertical offset from target note.
     * 
     * @return The vertical offset from target note.
     */
    public double getNoteTy() {
        return LimelightHelpers.getTY(limelightName);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Note TX", LimelightHelpers.getTX(limelightName));
        SmartDashboard.putNumber("Note TY", LimelightHelpers.getTY(limelightName));
        SmartDashboard.putNumber("Note TA", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putBoolean("Note detected", LimelightHelpers.getTV(limelightName));
    }

    /**
     * 
     * @return
     */
    public Optional<Rotation2d> getRotationalOverride() {
        if (LimelightHelpers.getTV(limelightName)) {
            return Optional.of(getNoteFieldRotation());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the {@link NeuralLimelight} subsystem instance
     * @return The {@link NeuralLimelight} subsystem instance
     */
    public static NeuralLimelight getSystem() {
        return neuralLimelight;
    }

}
