package raidzero.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {

    private CANSparkMax conveyorMotor;

    private SparkLimitSwitch beamBrake;

    private static Conveyor conveyor = new Conveyor();

    /**
     * Creates a new Conveyor object.
     */
    private Conveyor() {
        conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_ID, MotorType.kBrushless);

        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.enableVoltageCompensation(Constants.MAX_MOTOR_VOLTAGE);
        conveyorMotor.setSmartCurrentLimit(ConveyorConstants.CURRENT_LIMIT);
        conveyorMotor.setIdleMode(ConveyorConstants.IDLE_MODE);
        conveyorMotor.setInverted(ConveyorConstants.INVERSION);
    }

    /**
     * Sets the speed of the conveyor motor.
     * 
     * @param speed Speed to set in the range [-1, 1]
     */
    public void set(double speed) {
        conveyorMotor.set(speed);
    }

    /**
     * Stops the conveyor motor.
     */
    public void stop() {
        conveyorMotor.stopMotor();
    }

    /**
     * Gets the limit switch of the conveyor.
     * 
     * @return Conveyor {@link SparkLimitSwitch}
     */
    public SparkLimitSwitch getLimitSwitch() {
        return conveyorMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    }

    /**
     * Gets the beam break sensor.
     * 
     * @return Beam brake {@link SparkLimitSwitch}
     */
    public SparkLimitSwitch getBeamBrake() {
        return beamBrake;
    }

    /**
     * Gets the {@link Conveyor} subsystem.
     * 
     * @return {@link Conveyor} subsystem.
     */
    public static Conveyor getConveyor() {
        return conveyor;
    }

}