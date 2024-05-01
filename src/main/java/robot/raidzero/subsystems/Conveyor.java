package robot.raidzero.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    
    private CANSparkMax conveyorMotor;

    private SparkLimitSwitch beamBrake;

    private static Conveyor conveyor = new Conveyor();

    public Conveyor() {

    }

    public SparkLimitSwitch getBeamBrake() {
        return beamBrake;
    }

    public static Conveyor getConveyor() {
        return conveyor;
    }

}
