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

    public Conveyor() {
        conveyorMotor = new CANSparkMax(ConveyorConstants.CONVEYOR_ID, MotorType.kBrushless);

        conveyorMotor.restoreFactoryDefaults();
        conveyorMotor.enableVoltageCompensation(Constants.MAX_MOTOR_VOLTAGE);
        conveyorMotor.setSmartCurrentLimit(ConveyorConstants.CURRENT_LIMIT);
        conveyorMotor.setIdleMode(ConveyorConstants.IDLE_MODE);
        conveyorMotor.setInverted(ConveyorConstants.INVERSION);
    }

    public void set(double speed) {
        conveyorMotor.set(speed);
    }

    public void stop() {
        conveyorMotor.stopMotor();
    }

    public SparkLimitSwitch getLimitSwitch() {
        return conveyorMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    }

    public SparkLimitSwitch getBeamBrake() {
        return beamBrake;
    }

    public static Conveyor getConveyor() {
        return conveyor;
    }

}