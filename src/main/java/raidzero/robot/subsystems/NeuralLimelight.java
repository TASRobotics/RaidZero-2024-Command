package raidzero.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.LimelightHelper;

public class NeuralLimelight extends SubsystemBase {

    private final String limelightName = Constants.VisionConstants.NOTE_CAM_NAME;
    private static final NeuralLimelight neuralLimelight = new NeuralLimelight();

    private NeuralLimelight() {
        LimelightHelper.setLEDMode_ForceBlink(limelightName);
        LimelightHelper.setLEDMode_PipelineControl(limelightName);
    }

    public void initialize() {
        SmartDashboard.putNumber("Note TX", LimelightHelper.getTX(limelightName));
        SmartDashboard.putNumber("Note TY", LimelightHelper.getTY(limelightName));
        SmartDashboard.putNumber("Note TA", LimelightHelper.getTA(limelightName));
        SmartDashboard.putBoolean("Note detected", LimelightHelper.getTV(limelightName));
    }

    /**
     * Gets the horizontal offset from target note as a {@link Rotation2d}.
     * 
     * @return The horizontal offset from target note as a {@link Rotation2d}.
     */
    public Rotation2d getNoteRotation() {
        return Rotation2d.fromDegrees(LimelightHelper.getTX(limelightName));
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
        return LimelightHelper.getTV(limelightName);
    }

    /**
     * Gets the area of the note. (Relative to camera FOV...?)
     * 
     * @return
     */
    public double getNoteArea() {
        return LimelightHelper.getTA(limelightName);
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
        return LimelightHelper.getTX(limelightName);
    }

    /**
     * Gets the vertical offset from target note.
     * 
     * @return The vertical offset from target note.
     */
    public double getNoteTy() {
        return LimelightHelper.getTY(limelightName);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Note TX", LimelightHelper.getTX(limelightName));
        SmartDashboard.putNumber("Note TY", LimelightHelper.getTY(limelightName));
        SmartDashboard.putNumber("Note TA", LimelightHelper.getTA(limelightName));
        SmartDashboard.putBoolean("Note detected", LimelightHelper.getTV(limelightName));
    }

    /**
     * 
     * @return
     */
    public Optional<Rotation2d> getRotationalOverride() {
        if (LimelightHelper.getTV(limelightName)) {
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
