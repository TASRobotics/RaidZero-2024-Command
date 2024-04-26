package frc.raidzero.subsystems;




import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.raidzero.Constants;
import frc.raidzero.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {


    private enum ControlState {
        VELOCITY, PATHING, TESTING, PERCENT
    };

    private TalonFX mThrottle; 
    private TalonFX mAzimuth; 

    private CANcoder mAzimuthEncoder; 
    private double throttleStart;

    private VoltageOut throttleVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private VelocityVoltage throttleVelocityVoltageOut = new VelocityVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(SwerveConstants.kThrottleVelPIDSlot)
        .withUpdateFreqHz(SwerveConstants.kThrottlePIDUpdateHz);
    private VoltageOut azimuthVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private PositionVoltage azimuthPositionVoltageOut = new PositionVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(SwerveConstants.kAzimuthPositionPIDSlot)
        .withUpdateFreqHz(SwerveConstants.kAzimuthPIDUpdateHz);


    public static class PeriodicIO {
        public SwerveModuleState currentState = new SwerveModuleState(); 
        public SwerveModuleState desiredState = new SwerveModuleState();

        public SwerveModulePosition currentPosition = new SwerveModulePosition();

        public double outputThrottlePercentSpeed = 0.0;
        public double outputAzimuthPercentSpeed = 0.0; 

        public ControlState controlState = ControlState.VELOCITY;
    }


    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // @Override
    // public void onInit() {
    //     throw new RuntimeException("Cannot initialize SwerveModule by itself.");
    // }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int throttleId, int azimuthId, int azimuthEncoderId, double azimuthEncoderOffset) {
        mThrottle = new TalonFX(throttleId, Constants.kCANBusName);
        mThrottle.getConfigurator().apply(getThrottleConfig(), Constants.kLongCANTimeoutMs);

        mAzimuthEncoder = new CANcoder(azimuthEncoderId, Constants.kCANBusName);
        mAzimuthEncoder.getConfigurator().apply(getAzimuthEncoderConfig(azimuthEncoderOffset), Constants.kLongCANTimeoutMs);

        mAzimuth = new TalonFX(azimuthId, Constants.kCANBusName);
        mAzimuth.getConfigurator().apply(getAzimuthConfig(), Constants.kLongCANTimeoutMs);

        mPeriodicIO.currentState = new SwerveModuleState();
        mPeriodicIO.desiredState = new SwerveModuleState();
        mPeriodicIO.currentPosition = new SwerveModulePosition(); 
        stop();
    }

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {
        mPeriodicIO.controlState = ControlState.PERCENT;
        mPeriodicIO.outputThrottlePercentSpeed = 0.0;
        mPeriodicIO.outputAzimuthPercentSpeed = 0.0;
        resetToAbsolute();
        throttleStart = mThrottle.getPosition().refresh().getValue();
    }

    public double throttlePosTravelled(){
        return mThrottle.getPosition().refresh().getValue() - throttleStart;
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    @Override
    public void periodic() {
        // check if we should use motor position 
        Rotation2d wrappedRotation = Rotation2d.fromRadians(mAzimuthEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);
        Rotation2d rotation = Rotation2d.fromRotations(mAzimuth.getPosition().refresh().getValueAsDouble());

        // IMPORTANT!!!
        mPeriodicIO.currentState = new SwerveModuleState(
            mThrottle.getVelocity().refresh().getValue(), 
            rotation
        );

        mPeriodicIO.currentPosition = new SwerveModulePosition(
            mThrottle.getPosition().refresh().getValue(), 
            rotation
        );
    }

   
    public void stop() {
        testThrottleAndRotor(0, 0);
    }

     public void zero() {
        resetToAbsolute();
        mThrottle.setPosition(0.0);
    }

    /** Runs components in the submodule that have continuously changing inputs. */
    public void run() {
        switch (mPeriodicIO.controlState) {
            case VELOCITY:
                if(Math.abs(mPeriodicIO.desiredState.speedMetersPerSecond) > 0.1) {
                    mAzimuth.setControl(azimuthPositionVoltageOut.withPosition(getDesiredAzimuthOutputAngle(mPeriodicIO.desiredState.angle.getDegrees() / 360)));
                }
                mThrottle.setControl(throttleVelocityVoltageOut.withVelocity(-mPeriodicIO.desiredState.speedMetersPerSecond));
                break;
            case PATHING:
                break;
            case TESTING:
                mThrottle.setControl(throttleVoltageOut.withOutput(mPeriodicIO.outputThrottlePercentSpeed * Constants.kMaxMotorVoltage));
                mAzimuth.setControl(azimuthVoltageOut.withOutput(mPeriodicIO.outputAzimuthPercentSpeed * Constants.kMaxMotorVoltage));
                break;
            case PERCENT:
                if(Math.abs(mPeriodicIO.desiredState.speedMetersPerSecond) > 0.1) {
                    mAzimuth.setControl(azimuthPositionVoltageOut.withPosition(getDesiredAzimuthOutputAngle(mPeriodicIO.desiredState.angle.getDegrees() / 360)));
                }
                mThrottle.setControl(throttleVoltageOut.withOutput(mPeriodicIO.desiredState.speedMetersPerSecond * Constants.kMaxMotorVoltage));
                break;
        }
    }

    public void resetToAbsolute() {
        mAzimuth.setPosition(mAzimuthEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setClosedLoopState(SwerveModuleState state) {
        mPeriodicIO.controlState = ControlState.VELOCITY;
        state = SwerveModuleState.optimize(state, mPeriodicIO.currentState.angle);
        mPeriodicIO.desiredState = state;
    }

    // CHANGE 
    public void setOpenLoopState(SwerveModuleState state) {
        mPeriodicIO.controlState = ControlState.PERCENT;
        state = SwerveModuleState.optimize(state, mPeriodicIO.currentState.angle);
        // SwerveModuleState newState = SwerveModuleState.optimize(state, MathTools.wrapRotation2d(state.angle));
        mPeriodicIO.desiredState = state; 
    }

    public SwerveModuleState getState() {
        return mPeriodicIO.currentState;
    }

    public SwerveModulePosition getPosition() {
        return mPeriodicIO.currentPosition;
    }

    public TalonFX getAzimuthMotor() {
        return mAzimuth;
    }

    public CANcoder getAzimuthEncoder() {
        return mAzimuthEncoder;
    }

    public TalonFX getThrottleMotor() {
        return mThrottle; 
    }

    public PeriodicIO getPeriodicIO () {
        return mPeriodicIO;
    }

    /**
     * Get corrected Azimuth angle
     * 
     * @param angle desired angle to turn to (units: rotations)
     * @return corrected angle
     */
    public double getDesiredAzimuthOutputAngle(double angle) {
        // double currentAngle = mAzimuthEncoder.getPosition().refresh().getValueAsDouble();
        double currentAngle = mAzimuth.getPosition().refresh().getValueAsDouble();
        double delta = angle - currentAngle;
        delta = delta % 1.0;
        while (delta > 0.5)
            delta -= 1.0;
        while (delta < -0.5)
            delta += 1.0;

        SmartDashboard.putNumber("corrected azimuth output angle", currentAngle + delta);

        return currentAngle + delta;
    }

    public void testThrottleAndRotor(double throttleOutput, double rotorOutput) {
        mPeriodicIO.controlState = ControlState.TESTING;

        mPeriodicIO.outputThrottlePercentSpeed = throttleOutput;
        mPeriodicIO.outputAzimuthPercentSpeed = rotorOutput;
    }

    
    private TalonFXConfiguration getThrottleConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(SwerveConstants.kThrottleInversion);
        motorOutputConfigs.withNeutralMode(SwerveConstants.kThrottleNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        config.withCurrentLimits(SwerveConstants.kThrottleCurrentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(SwerveConstants.kThrottleRotToWheelRot * SwerveConstants.kThrottleWheelRotToMeters);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKA(SwerveConstants.kThrottle_kA);
        slot0Configs.withKV(SwerveConstants.kThrottle_kV);
        slot0Configs.withKP(SwerveConstants.kThrottle_kP);
        slot0Configs.withKI(SwerveConstants.kThrottle_kI);
        slot0Configs.withKD(SwerveConstants.kThrottle_kD);
        config.withSlot0(slot0Configs);

        // Closed Loop General Configs
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        config.withClosedLoopGeneral(closedLoopGeneralConfigs);

        // Open Loop Ramps Configs
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        // openLoopRampsConfigs.withVoltageOpenLoopRampPeriod(SwerveConstants.kOpenLoopRampRate);
        config.withOpenLoopRamps(openLoopRampsConfigs);

        return config; 
    }

    private TalonFXConfiguration getAzimuthConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(SwerveConstants.kAzimuthInversion);
        motorOutputConfigs.withNeutralMode(SwerveConstants.kAzimuthNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        config.withCurrentLimits(SwerveConstants.kAzimuthCurrentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(1 / SwerveConstants.kAzimuthReduction);
        config.withFeedback(feedbackConfigs);

        // Position PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKP(SwerveConstants.kAzimuth_kP);
        slot0Configs.withKI(SwerveConstants.kAzimuth_kI);
        slot0Configs.withKD(SwerveConstants.kAzimuth_kD);
        config.withSlot0(slot0Configs);

        return config; 
    }

    private CANcoderConfiguration getAzimuthEncoderConfig(double angleOffset) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Magnet Sensor Configuration
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.withAbsoluteSensorRange(SwerveConstants.kAzimuthEncoderRange);
        magnetSensorConfigs.withSensorDirection(SwerveConstants.kAzimuthEncoderDirection);
        magnetSensorConfigs.withMagnetOffset(angleOffset);
        config.withMagnetSensor(magnetSensorConfigs);

        return config;
    }
}