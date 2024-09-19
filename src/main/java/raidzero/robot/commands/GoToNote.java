package raidzero.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.CommandSwerveDrivetrain;
import raidzero.robot.subsystems.NeuralLimelight;

public class GoToNote extends Command {

    CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.system();

    NeuralLimelight neuralLL = NeuralLimelight.getSystem();
    
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public GoToNote() {}

    @Override
    public void execute() {
        swerve.setControl(AutoRequest.withSpeeds(
            ChassisSpeeds.fromRobotRelativeSpeeds(
                0.0,
                0.3,
                0.0,
                swerve.getPigeon2().getRotation2d()
            )
        ));
    }

    @Override
    public boolean isFinished() { // TY < 17 deg
        return neuralLL.getNoteTy() > 17.0;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
