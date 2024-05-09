package raidzero.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {

    public enum States {
        TORQUE,
        FF
    }

    private TalonFX intakeMotor;

    private final VoltageOut intakeVoltageOut = new VoltageOut(0.0)
        .withEnableFOC(Constants.ENABLE_FOC);

    private TorqueCurrentFOC intakeCurrent = new TorqueCurrentFOC(0.0);

    private SparkLimitSwitch beamBreak;

    private Timer beamBreakTimer = new Timer();

    private boolean limitTriggered = false;

    private static Intake intake = new Intake();

    /**
     * Creates a new Intake object.
     */
    public Intake() {

        intakeMotor = new TalonFX(IntakeConstants.INTAKE_ID);
        intakeMotor.getConfigurator().apply(getConfigs());

        beamBreak = Conveyor.getConveyor().getBeamBrake();
        beamBreak.enableLimitSwitch(false);

    }

    @Override
    public void periodic() {
        boolean beamBreakPressed = beamBreak.isPressed();

        if (beamBreakPressed) {
            beamBreakTimer.start();
        } else {
            beamBreakTimer.reset();
        }

        limitTriggered = beamBreakTimer.get() > IntakeConstants.BEAM_BRAKE_TIME && beamBreak.isPressed();
    }

    /**
     * Sets the state of the intake.
     * 
     * @param state Control mode (TORQUE or FF)
     * @param value Value (Current if TORQUE, Percent if FF ([-1.0, 1.0]))
     */
    public void setState(States state, double value) {
        if (state == States.FF) {
            intakeMotor.setControl(intakeVoltageOut.withOutput(value * Constants.MAX_MOTOR_VOLTAGE));
        } else if (state == States.TORQUE) {
            intakeMotor.setControl(intakeCurrent.withOutput(value));
        }
    }

    /**
     * Checks if a note is present in Intake.
     * 
     * @return True if a note is present, false otherwise.
     */
    public boolean notePresent() {
        return limitTriggered;
    }

    /**
     * Stops the motors.
     */
    public void stopMotors() {
        intakeMotor.stopMotor();
    }

    /**
     * Gets the {@link Intake} subsystem.
     * 
     * @return {@link Intake} subsystem.
     */
    public static Intake getIntake() {
        return intake;
    }

    //* Configurations
    /**
     * Configures the TalonFX for the intake motor.
     * 
     * @return {@link TalonFXConfiguration} for the intake motor.
     */
    private TalonFXConfiguration getConfigs() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(IntakeConstants.LEADER_INVERSION)
            .withNeutralMode(IntakeConstants.NEUTRAL_MODE);

        CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(IntakeConstants.SUPPLY_CURRENT_ENABLE)
            .withSupplyCurrentThreshold(IntakeConstants.SUPPLY_CURRENT_THRESHOLD)
            .withSupplyTimeThreshold(IntakeConstants.SUPPLY_TIME_THRESHOLD);

        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs()
            .withVoltageOpenLoopRampPeriod(IntakeConstants.VOLTAGE_RAMP_RATE);

        config.withMotorOutput(motorOutputConfigs)
            .withCurrentLimits(currentLimitConfigs)
            .withOpenLoopRamps(openLoopRampsConfigs);

        return config;
    }

}