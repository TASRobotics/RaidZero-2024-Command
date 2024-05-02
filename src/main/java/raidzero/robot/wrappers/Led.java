package raidzero.robot.wrappers;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

    private static I2C leds;
    private static byte[] data = new byte[1];

    private static Led led = new Led();

    public Led() {
        glowPattern(0);

        leds = new I2C(Port.kOnboard, 4);
        glowPattern(0);
    }

    public void glowPattern(int patternId) {
        data[0] = (byte) patternId;
        if (patternId < 256)
            leds.transaction(data, 1, new byte[1], 0);
        else
            return;
    }

    public void stop() {
        glowPattern(1);
    }

    public static Led getLed() {
        return led;
    }
    
}
