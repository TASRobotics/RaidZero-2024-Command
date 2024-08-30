package raidzero.robot.subsystems;

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
        return this.getNoteRotation().plus(CommandSwerveDrivetrain.system().getPigeon2().getRotation2d());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Note TX", LimelightHelper.getTX(limelightName));
        SmartDashboard.putNumber("Note TY", LimelightHelper.getTY(limelightName));
        SmartDashboard.putNumber("Note TA", LimelightHelper.getTA(limelightName));
        SmartDashboard.putBoolean("Note detected", LimelightHelper.getTV(limelightName));
    }

    /**
     * Gets the {@link NeuralLimelight} subsystem instance
     * @return The {@link NeuralLimelight} subsystem instance
     */
    public NeuralLimelight getSystem() {
        return neuralLimelight;
    }

}
