package raidzero.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.Constants;

public class TeleopSwerve extends Command {
    
    private raidzero.robot.subsystems.Swerve swerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private double translationVal;
    private double strafeVal;
    private double rotationVal;

    /**
     * Creates a new TeleopSwerve command.
     * 
     * @param translationSup Translatoin value supplier
     * @param strafeSup Strafe value supplier
     * @param rotationSup Rotation value supplier
     * @param robotCentricSup Robot-centric mode supplier
     */
    public TeleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup,DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve = raidzero.robot.subsystems.Swerve.getInstance();
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        swerve.drive(
            new Translation2d(translationVal * translationVal * translationVal, strafeVal * strafeVal * strafeVal).times(Constants.Swerve.MAX_VEL_MPS),
            rotationVal * rotationVal * rotationVal * Constants.Swerve.MAX_ANGULAR_VEL_RPS,
            true,
            false
        );
    }

}
