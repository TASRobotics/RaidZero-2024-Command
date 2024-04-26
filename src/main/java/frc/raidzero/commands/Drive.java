package frc.raidzero.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.raidzero.subsystems.Swerve;

public class Drive extends Command {
    private final CommandXboxController controller;
    private final Swerve swerve;

    public Drive(CommandXboxController controller) {
        this.controller = controller;
        this.swerve = Swerve.getInstance(); // Instantiate the Swerve object
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Add initialization code here
    }

    @Override
    public void execute() {
        // Add code to drive the robot based on joystick input from the Xbox controller
        double forward = controller.getLeftY();
        double strafe = controller.getLeftX();
        double rotation = controller.getRightX();
        swerve.teleopDrive(forward, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        // Add code to stop the robot when the command ends
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        // Add code to determine when the command should end
        return false;
    }
}
