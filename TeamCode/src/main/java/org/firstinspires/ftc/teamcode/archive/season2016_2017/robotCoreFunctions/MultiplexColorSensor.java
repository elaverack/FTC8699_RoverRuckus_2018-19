package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;

/**
 * Created by Chandler on 1/25/2017.
 */
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

public class MultiplexColorSensor {

    // Registers
    static final int ENABLE = 0x80;
    static final int ATIME = 0x81;
    static final int CONTROL = 0x8F;
    static final int ID = 0x92;
    static final int STATUS = 0x93;
    static final int CDATAL = 0x94;

    // Default I2C address for multiplexer. The address can be changed to any
    // value from 0x70 to 0x77, so this line would need to be changed if a
    // non-default address is to be used.
    static final I2cAddr MUX_ADDRESS = new I2cAddr(0x70);
    // I2C address for color sensor
    static final I2cAddr ADA_ADDRESS = new I2cAddr(0x29);
    public static int GAIN_1X = 0x00;
    public static int GAIN_4X = 0x01;
    public static int GAIN_16X = 0x02;
    public static int GAIN_60X = 0x03;
    private I2cDevice mux;
    private I2cDeviceSynch muxReader;
    // Only one color sensor is needed in code as the multiplexer switches
    // between the physical sensors
    private byte[] adaCache;
    private I2cDevice ada;
    private I2cDeviceSynch adaReader;
    private int[] sensorPorts;
    private int lastPort;

    public static int ATIME_SLOWEST = 0x00;
    public static int ATIME_SLOW = 0xC0;
    public static int ATIME_MED = 0xD5;
    public static int ATIME_FAST = 0xF6;
    public static int ATIME_FASTEST = 0xFF;

    /**
     * Initializes Adafruit color sensors on the specified ports of the I2C
     * multiplexer.
     *
     * @param hardwareMap  hardwareMap from OpMode
     * @param muxName      Configuration name of I2CDevice for multiplexer
     * @param colorName    Configuration name of I2CDevice for color sensor
     * @param ports        Out ports on multiplexer with color sensors attached
     * @param milliSeconds Integration time in milliseconds
     * @param gain         Gain (GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X)
     */
    public MultiplexColorSensor(HardwareMap hardwareMap, String muxName, String colorName, int[] ports, double milliSeconds, int gain) {

        sensorPorts = ports;

        mux = hardwareMap.i2cDevice.get(muxName);
        muxReader = new I2cDeviceSynchImpl(mux, MUX_ADDRESS, false);
        muxReader.engage();

        // Loop over the ports activating each color sensor
        for (int i = 0; i < sensorPorts.length; i++) {
            // Write to given output port on the multiplexer
            muxReader.write8(0x00, sensorPorts[i]+1, true);

            ada = hardwareMap.i2cDevice.get(colorName);
            adaReader = new I2cDeviceSynchImpl(ada, ADA_ADDRESS, false);
            adaReader.engage();

            final int time = integrationByte(milliSeconds);
            adaReader.write8(ENABLE, 0x03, true);  // Power on and enable ADC
            adaReader.read8(ID);                   // Read device ID
            adaReader.write8(CONTROL, gain, true); // Set gain
            adaReader.write8(ATIME, time, true);   // Set integration time
        }
    }

    /**
     * Set the integration time on all the color sensors
     *
     * @param milliSeconds Time in millseconds
     */
    public void setIntegrationTime(double milliSeconds) {
        int val = integrationByte(milliSeconds);

        for (int i = 0; i < sensorPorts.length; i++) {
            muxReader.write8(0x0, 1 << sensorPorts[i], true);
            adaReader.write8(ATIME, val, true);
        }
    }

    private int integrationByte(double milliSeconds) {
        return (4);
    }

    // Un-needed?
    public void startPolling() {
        for (int i = 0; i < sensorPorts.length; i++) {
            muxReader.write8(0x0, 1 << sensorPorts[i], true);
            adaReader.read8(STATUS);
        }
    }

    /**
     * Retrieve the color read by the given color sensor
     *
     * @param port Port on multiplexer of given color sensor
     * @return Array containing the Clear, Red, Green, and Blue color values
     */
    public int[] getCRGB(int port) {
        // Write to I2C port on the multiplexer
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x0, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }

        // Read color registers
        adaCache = adaReader.read(CDATAL, 8);

        // Combine high and low bytes
        int[] crgb = new int[4];
        for (int i = 0; i < 4; i++) {
            crgb[i] = (adaCache[2 * i] & 0xFF) + (adaCache[2 * i + 1] & 0xFF) * 256;
        }

        return crgb;
    }

    public int red(int port) {
        final int RED_REGISTER = 0x96;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return Math.abs(adaReader.read8(RED_REGISTER));
    }

    public int green(int port) {
        final int GREEN_REGISTER = 0x98;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return Math.abs(adaReader.read8(GREEN_REGISTER));
    }

    public int blue(int port) {
        final int BLUE_REGISTER = 0x9A;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return Math.abs(adaReader.read8(BLUE_REGISTER));
    }

    public int clear(int port) {
        final int CLEAR_REGISTER = 0x94;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return Math.abs(adaReader.read8(CLEAR_REGISTER));
    }

    public int color(int port) {
        final int RED_REGISTER = 0x96;
        final int GREEN_REGISTER = 0x98;
        final int BLUE_REGISTER = 0x9A;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        final int r = Math.abs(adaReader.read8(RED_REGISTER));
        final int g = Math.abs(adaReader.read8(GREEN_REGISTER));
        final int b = Math.abs(adaReader.read8(BLUE_REGISTER));
        return Color.rgb(r, g, b);
    }

    public void disable (int port) {
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        adaReader.disengage();
    }

    public void setEnable (int port) {
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            muxReader.write8(0x00, 1 << port, true);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        adaReader.engage();
    }

}