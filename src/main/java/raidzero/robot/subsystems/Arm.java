package raidzero.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    private TalonFX leftleader = new TalonFX(ArmConstants.LEFT_LEADER_ID, Constants.CANBUS_ID);
    private TalonFX rightFollower = new TalonFX(ArmConstants.RIGHT_FOLLOWER_ID, Constants.CANBUS_ID);

    private CANcoder encoder = new CANcoder(ArmConstants.ENCODER_ID, Constants.CANBUS_ID);

    private VoltageOut voltageOut = new VoltageOut(0.0)
        .withEnableFOC(Constants.ENABLE_FOC);

    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.ENABLE_FOC)
        .withSlot(ArmConstants.POSITION_PID_SLOT)
        .withUpdateFreqHz(ArmConstants.PID_UPDATE_HZ);
}
