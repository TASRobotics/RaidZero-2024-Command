package raidzero.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    public enum LedMode {
        Current,
        Off,
        Blink,
        On
    }

    public enum CameraMode {
        Vision,
        Driver
    }

    public enum StreamMode {
        Standard,
        PIP_Main,
        PIP_Secondary
    }

    private NetworkTable nt;
    private String ntName;
    
    public Limelight(String ntName) {
        this.ntName = ntName;

        setLedMode(LedMode.Off);
        setPipeline(0);
    }

    /**
	 * Sets stream mode for Limelight.
	 * 
	 * @param mode Stream mode for Limelight
	 */
	public void setStreamMode(StreamMode mode) {
		getValue("stream").setNumber(mode.ordinal());
	}

    /**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

    /**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode Light mode for Limelight.
	 */
	public void setLedMode(LedMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

    /**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(0.0);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.0);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.0);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.0);
	}

	/**
	 * Gets target latency (ms).
	 * 
	 * @return Target latency.
	 */
	public double getTl() {
		return getValue("tl").getDouble(0.0);
	}

	/**
	 * Returns an array of corner x-coordinates.
	 *
	 * Note: Enable "send contours" in the "Output" tab to stream corner
	 * coordinates.
	 * 
	 * @return Corner x-coordinates.
	 */
	public double[] getTCornX() {
		return getValue("tcornx").getDoubleArray(new double[] {});
	}

	/**
	 * Returns an array of corner y-coordinates.
	 *
	 * Note: Enable "send contours" in the "Output" tab to stream corner
	 * coordinates.
	 *
	 * @return Corner y-coordinates.
	 */
	public double[] getTCornY() {
		return getValue("tcorny").getDoubleArray(new double[] {});
	}

    /**
	 * Gets whether a target is detected by the Limelight.
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean hasTarget() {
		return getValue("tv").getDouble(0.0) > 0.0;
	}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight X Value", getTx());
    }

    private NetworkTableEntry getValue(String key) {
		if (nt == null) {
			nt = NetworkTableInstance.getDefault().getTable(ntName);
		}
		return nt.getEntry(key);
	}
    
}
