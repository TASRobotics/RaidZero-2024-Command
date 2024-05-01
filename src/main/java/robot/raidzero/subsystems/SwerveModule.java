package robot.raidzero.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import robot.raidzero.Constants;
import robot.raidzero.Constants.Swerve;
import robot.raidzero.lib.math.Conversions;

public class SwerveModule {
    
    private TalonFX throttle;
    private TalonFX azimuth;

    private CANcoder azimuthEncoder;
    private double throttleStart;

    private final DutyCycleOut throttleDutyCycleOut = new DutyCycleOut(0.0)
        .withEnableFOC(Constants.ENABLE_FOC);

    private final VelocityVoltage throttleVelocity = new VelocityVoltage(0.0)
        .withEnableFOC(Constants.ENABLE_FOC)
        .withSlot(Constants.Swerve.THROTTLE_VEL_PID_SLOT)
        .withUpdateFreqHz(Constants.Swerve.THROTTLE_PID_UPDATE_HZ);

    private final PositionVoltage azimuthPosition = new PositionVoltage(0.0)
        .withEnableFOC(Constants.ENABLE_FOC)
        .withSlot(Constants.Swerve.AZIMUTH_POS_PID_SLOT)
        .withUpdateFreqHz(Constants.Swerve.AZIMUTH_PID_UPDATE_HZ);

    private final SimpleMotorFeedforward throttleFF = new SimpleMotorFeedforward(
        Constants.Swerve.THROTTLE_KS,
        Constants.Swerve.THROTTLE_KV,
        Constants.Swerve.THROTTLE_KA
    );

    /**
     * Constructs a new SwerveModule object
     * 
     * @param throttleId Throttle motor ID
     * @param azimuthId Azimuth motor ID
     * @param azimuthEncoderId Azimuth encoder ID
     * @param azimuthOffset Azimuth offset
     */
    public SwerveModule(int throttleId, int azimuthId, int azimuthEncoderId, double azimuthOffset) {
        throttle = new TalonFX(throttleId, Constants.CANBUS_ID);
        azimuth = new TalonFX(azimuthId, Constants.CANBUS_ID);

        azimuthEncoder = new CANcoder(azimuthEncoderId, Constants.CANBUS_ID);
        
        throttle.getConfigurator().apply(getThrottleConfig());    
        azimuth.getConfigurator().apply(getAzimuthConfig());
        azimuthEncoder.getConfigurator().apply(getAzimuthEncoderConfig(azimuthOffset));

        throttleStart = throttle.getPosition().refresh().getValue();
        stopMotors();
    }

    /**
     * Sets the desired state of the module.
     * 
     * @param desiredState Desired state
     * @param isOpenLoop Open loop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        azimuth.setControl(azimuthPosition.withPosition(desiredState.angle.getRotations()));

        if (isOpenLoop) {
            throttleDutyCycleOut.Output = desiredState.speedMetersPerSecond / Constants.Swerve.REAL_MAX_VEL_MPS;
            throttle.setControl(throttleDutyCycleOut);
        } else {
            throttleVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.Swerve.WHEEL_CIRCUMFERENCE);
            throttleVelocity.FeedForward = throttleFF.calculate(desiredState.speedMetersPerSecond);
            throttle.setControl(throttleVelocity);
        }
    }

    /**
     * Gets azimuth encoder value.
     * 
     * @return Azimuth encoder rotation as {@link Rotation2d}
     */
    public Rotation2d getCANCoderRotations() {
        return Rotation2d.fromRotations(azimuthEncoder.getAbsolutePosition().getValue());
    }
    
    /**
     * Resets azimuth motor to azimuth encoder position
     */
    public void resetToAbsolute() {
        azimuth.setPosition(azimuthEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Gets the current state of the module.
     * 
     * @return Current {@link SwerveModuleState}
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(throttle.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(azimuth.getPosition().getValue()));
    }

    /**
     * Gets the current position of the module.
     * 
     * @return Current {@link SwerveModulePosition}
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(throttle.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(azimuth.getPosition().getValue()));
    }

    /**
     * Stops all motors
     */
    public void stopMotors() {
        azimuth.stopMotor();
        throttle.stopMotor();
    }


    //* Configurations
    /**
     * Configures the TalonFX for the throttle motor.
     * 
     * @return {@link TalonFXConfiguration} for the throttle motor.
     */
    private TalonFXConfiguration getThrottleConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(Swerve.THROTTLE_INVERSION)
            .withNeutralMode(Swerve.THROTTLE_NEUTRAL_MODE);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(Swerve.THROTTLE_ROT_TO_WHEEL_ROTATION * Swerve.THROTTLE_WHEEL_ROT_TO_METERS);

        Slot0Configs slot0Configs = new Slot0Configs()
            .withKA(Swerve.THROTTLE_KA)
            .withKV(Swerve.THROTTLE_KV)
            .withKP(Swerve.THROTTLE_KP)
            .withKI(Swerve.THROTTLE_KI)
            .withKD(Swerve.THROTTLE_KD);

        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();

        config.withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(Swerve.THROTTLE_CURRENT_LIMIT_CONFIGS)
            .withFeedback(feedbackConfigs)
            .withSlot0(slot0Configs)
            .withClosedLoopGeneral(closedLoopGeneralConfigs)
            .withOpenLoopRamps(openLoopRampsConfigs);

        return config;
    }

    /**
     * Configures the TalonFX for the azimuth motor.
     * 
     * @return {@link TalonFXConfiguration} for the azimuth motor.
     */
    private TalonFXConfiguration getAzimuthConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(Swerve.AZIMUTH_INVERSION)
            .withNeutralMode(Swerve.AZIMUTH_NEUTRAL_MODE);

        config.withCurrentLimits(Swerve.AZIMUTH_CURRENT_LIMIT_CONFIGS);
        
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / Swerve.AZIMUTH_REDUCTION);

        Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(Swerve.AZIMUTH_KP)
            .withKI(Swerve.AZIMUTH_KI)
            .withKD(Swerve.AZIMUTH_KD);

        config.withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(Swerve.AZIMUTH_CURRENT_LIMIT_CONFIGS)
            .withFeedback(feedbackConfigs)
            .withSlot0(slot0Configs);

        return config;
    }

    /**
     * Configures the CANCoder for the azimuth encoder.
     * 
     * @param angleOffset Angle offset of the azimuth encoder.
     * @return {@link CANcoderConfiguration} for the azimuth encoder.
     */
    private CANcoderConfiguration getAzimuthEncoderConfig(double angleOffset) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
            .withAbsoluteSensorRange(Swerve.AZIMUTH_ENCODER_RANGE)
            .withSensorDirection(Swerve.AZIMUTH_ENCODER_DIRECTION)
            .withMagnetOffset(angleOffset);

        config.withMagnetSensor(magnetSensorConfigs);

        return config;
    }

}
