package raidzero.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.Swerve;
import raidzero.robot.subsystems.Vision;

public class FollowPath extends Command {

    private Swerve swerve = Swerve.getInstance();
    private Vision vision = Vision.getInstance();

    private PPHolonomicDriveController ppController = swerve.getPpController();
    private ProfiledPIDController aimAssistController = swerve.getAimAssistYController();
    private PIDController snapController = swerve.getSnapController();
    private Alliance alliance = swerve.getAlliance();

    private boolean overridePathingRotationSpeakerAim, overridePathingRotationNoteAim;

    private PathPlannerTrajectory path;

    private Timer timer;

    public FollowPath(PathPlannerTrajectory path, boolean isFirstPath, boolean overridePathingRotationSpeakerAim, boolean overridePathingRotationNoteAim) {
        this.path = path;
        this.overridePathingRotationSpeakerAim = overridePathingRotationSpeakerAim;
        this.overridePathingRotationNoteAim = overridePathingRotationNoteAim;
        
        if (isFirstPath) {
            swerve.setPose(
                new Pose2d(
                    path.getInitialState().positionMeters,
                    path.getInitialState().heading
                )
            );
        }
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerTrajectory.State state = (PathPlannerTrajectory.State) path.sample(timer.get());

        ppController.setEnabled(true);

        Pose2d currPose = swerve.getPose();
        System.out.println("degrees: " + currPose.getRotation().getDegrees());
        System.out.println("x: " + currPose.getX());
        System.out.println("y: " + currPose.getY());

        ChassisSpeeds desiredSpeeds = ppController.calculateRobotRelativeSpeeds(/*mCurrentAutoPose*/ currPose, state);
        SmartDashboard.putNumber("Desired State X", state.getTargetHolonomicPose().getX());
        SmartDashboard.putNumber("Desired State Y", state.getTargetHolonomicPose().getY());

        SmartDashboard.putNumber("error", ppController.getPositionalError());

        if(overridePathingRotationSpeakerAim && vision.getSpeakerAngle(alliance) != null) {
            double omega = snapController.calculate(vision.getSpeakerAngle(alliance).getDegrees() - swerve.getPigeon().getRotation2d().getDegrees());
            // double omega = (vision.getSpeakerAngle(alliance).getDegrees() - swerve.getPigeon().getRotation2d().getDegrees()) * 0.15;
            desiredSpeeds = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, omega);
        }
        if(overridePathingRotationNoteAim && vision.hasNote()) {
            double omega = aimAssistController.calculate(vision.getNoteXFilter().lastValue(), 0.0);
            desiredSpeeds = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, omega);
        }

        swerve.setModuleStates(Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(desiredSpeeds));
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= path.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
    
}
