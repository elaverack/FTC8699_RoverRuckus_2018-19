package org.firstinspires.ftc.teamcode.robotHandlers.I2cSupport;

/**
 * Created by Chandler on 1/25/2017.
 */
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.teamcode.robotHandlers.I2cSupport.TCS34725_ColorSensor;
import org.firstinspires.ftc.teamcode.robotHandlers.I2cSupport.Wire;

public class BETA_MultiplexedColorSensors {

    // Registers
    static final int
            ENABLE = 0x80,
            ATIME = 0x81,
            CONTROL = 0x8F,
            ID = 0x92,
            STATUS = 0x93,
            CDATAL = 0x94;

    // Default I2C address for multiplexer. The address can be changed to any
    // value from 0x70 to 0x77, so this line would need to be changed if a
    // non-default address is to be used.
    static final I2cAddr
            MUX_ADDRESS = new I2cAddr(0x70),
    // I2C address for color sensor
            ADA_ADDRESS = new I2cAddr(0x29);
    public static enum GAIN {_1X, _4X, _16X, _60X}
    private static final int
            GAIN_1X =  0,
            GAIN_4X =  1,
            GAIN_16X = 2,
            GAIN_60X = 3;
    private I2cDevice mux;
    private I2cDeviceSynch muxReader;
    // Only one color sensor is needed in code as the multiplexer switches
    // between the physical sensors
    private byte[] adaCache;
    private I2cDevice ada;
    private I2cDeviceSynch adaReader;
    private int lastPort;

    public static enum ATIME {SLOWEST, SLOW, MED, FAST, FASTEST}
    private static final int
            ATIME_SLOWEST = 0x00,
            ATIME_SLOW = 0xC0,
            ATIME_MED = 0xD5,
            ATIME_FAST = 0xF6,
            ATIME_FASTEST = 0xFF;

    // The values to write to the mux when switching sensors
    //private final int[] muxPortWrites = {224, 226, 228, 230, 232, 234, 236, 238};
    private final int[] muxPortWrites = {1, 1<<1, 1<<2, 1<<3, 1<<4, 1<<5, 1<<6, 1<<7};

    Wire MUX;
    TCS34725_ColorSensor ADA;

    /**
     * Initializes Adafruit color sensors on the specified ports of the I2C
     * multiplexer.
     *
     * @param hardwareMap  hardwareMap from OpMode
     * @param muxName      Configuration name of I2CDevice for multiplexer
     * @param colorName    Configuration name of I2CDevice for color sensor
     * @param NoPorts      Number of ports on multiplexer, default is 8
     * @param atime        ATIME -- or speed of sensor. NOTE: You trade of accuracy when you speed up the sensor.
     * @param gain         Gain (GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X)
     */
    public BETA_MultiplexedColorSensors(HardwareMap hardwareMap, String muxName, String colorName, int NoPorts, ATIME atime, GAIN gain) {

        MUX = new Wire(hardwareMap, muxName, MUX_ADDRESS.get8Bit());
        //mux = hardwareMap.i2cDevice.get(muxName);
        //muxReader = new I2cDeviceSynchImpl(mux, MUX_ADDRESS, false);
        //muxReader.engage();

        // Loop over the ports activating each color sensor
        for (int i = 0; i < NoPorts; i++) {
            // Write to given output port on the multiplexer
            //muxReader.write8(0x00, muxPortWrites[i], true);
            MUX.write(0x00, muxPortWrites[i]);

            //ada = hardwareMap.i2cDevice.get(colorName);
            //adaReader = new I2cDeviceSynchImpl(ada, ADA_ADDRESS, false);
            //adaReader.engage();
            ADA = new TCS34725_ColorSensor(hardwareMap, colorName);

            //adaReader.write8(ENABLE, 0x03, true);  // Power on and enable ADC
            //adaReader.read8(ID);                   // Read device ID
            //adaReader.write8(CONTROL, gainToInt(gain), true); // Set gain
            //adaReader.write8(ATIME, aTimeToInt(atime), true);   // Set integration time

            //ADA.setIntegrationTime(10);

        }
    }

    public BETA_MultiplexedColorSensors(HardwareMap hardwareMap, String muxName, String colorName, ATIME atime, GAIN gain) {
        MUX = new Wire(hardwareMap, muxName, MUX_ADDRESS.get8Bit());
        for (int i = 0; i < 8; i++) {
            MUX.write(0x00, muxPortWrites[i]);
            ADA = new TCS34725_ColorSensor(hardwareMap, colorName);
        }
    }

    public BETA_MultiplexedColorSensors(HardwareMap hardwareMap, String muxName, String colorName, int NoPorts) {

        MUX = new Wire(hardwareMap, muxName, MUX_ADDRESS.get8Bit());
        //mux = hardwareMap.i2cDevice.get(muxName);
        //muxReader = new I2cDeviceSynchImpl(mux, MUX_ADDRESS, false);
        //muxReader.engage();

        // Loop over the ports activating each color sensor
        for (int i = 0; i < NoPorts; i++) {
            // Write to given output port on the multiplexer
            //muxReader.write8(0x00, muxPortWrites[i], true);
            MUX.write(0x00, muxPortWrites[i]);

            //ada = hardwareMap.i2cDevice.get(colorName);
            //adaReader = new I2cDeviceSynchImpl(ada, ADA_ADDRESS, false);
            //adaReader.engage();
            ADA = new TCS34725_ColorSensor(hardwareMap, colorName);

            //adaReader.write8(ENABLE, 0x03, true);  // Power on and enable ADC
            //adaReader.read8(ID);                   // Read device ID
            //adaReader.write8(CONTROL, gainToInt(gain), true); // Set gain
            //adaReader.write8(ATIME, aTimeToInt(atime), true);   // Set integration time

            //ADA.setIntegrationTime(10);

        }
    }

    /*public void setATIME (ATIME atime) {
        for (int i = 0; i < 8; i++) {
            // Write to given output port on the multiplexer
            muxReader.write8(0x00, muxPortWrites[i], true);
            adaReader.write8(ATIME, aTimeToInt(atime), true);   // Set integration time
        }
    }

    public void setATIME (ATIME atime, int NoPorts) {
        for (int i = 0; i < NoPorts; i++) {
            // Write to given output port on the multiplexer
            muxReader.write8(0x00, muxPortWrites[i], true);
            adaReader.write8(ATIME, aTimeToInt(atime), true);   // Set integration time
        }
    }

    private int gainToInt (GAIN gain) {
        switch (gain) {
            case _1X:
                return GAIN_1X;
            case _4X:
                return GAIN_4X;
            case _16X:
                return GAIN_16X;
            case _60X:
                return GAIN_60X;
            default:
                return 0;
        }
    }

    private int aTimeToInt (ATIME atime) {
        switch (atime) {
            case SLOWEST:
                return ATIME_SLOWEST;
            case SLOW:
                return ATIME_SLOW;
            case MED:
                return ATIME_MED;
            case FAST:
                return ATIME_FAST;
            case FASTEST:
                return ATIME_FASTEST;
            default:
                return 0;
        }
    }*/

    /**
     * Retrieve the color read by the given color sensor
     *
     * @param port Port on multiplexer of given color sensor
     * @return Array containing the Clear, Red, Green, and Blue color values
     */
    public int[] getCRGB(int port) {
        // Write to I2C port on the multiplexer
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            //muxReader.write8(0x0, muxPortWrites[port], true);
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }

        // Read color registers
        //adaCache = adaReader.read(CDATAL, 8);

        // Combine high and low bytes
        int[] crgb = new int[4];
        //for (int i = 0; i < 4; i++) {
        //    crgb[i] = (adaCache[2 * i] & 0xFF) + (adaCache[2 * i + 1] & 0xFF) * 256;
        //}
        crgb[0] = ADA.clearColor();
        crgb[1] = ADA.redColor();
        crgb[1] = ADA.greenColor();
        crgb[1] = ADA.blueColor();

        return crgb;


    }

    public int red(int port) {
        //final int RED_REGISTER = 0x96;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            //muxReader.write8(0x00, muxPortWrites[port], true);
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        //adaCache = adaReader.read(RED_REGISTER, 2);
        //return ((adaCache[0] & 0xFF) + (adaCache[1] & 0xFF) * 256);
        return ADA.redColor();
    }

    public int green(int port) {
        //final int GREEN_REGISTER = 0x98;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return ADA.greenColor();
    }

    public int blue(int port) {
        //final int BLUE_REGISTER = 0x9A;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return ADA.blueColor();
    }

    public int clear(int port) {
        //final int CLEAR_REGISTER = 0x94;
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return ADA.clearColor();
    }

    public int color(int port) {
        final int r = red(port), g = green(port), b = blue(port); return Color.rgb(r, g, b);
    }

    public int colorTemp (int port) {
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return ADA.colorTemp();
    }

    public boolean getStatus (int port) {
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        return ADA.isIdOk();
    }

    /*public void disable (int port) {
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        adaReader.disengage();
    }

    public void enable (int port) {
        if (port != lastPort) {      // might speed things up a tad, as we only set the mux when changing between sensors
            MUX.write(0x00, muxPortWrites[port]);
            lastPort = port;        // not on consecutive reads of the same sensor
        }
        adaReader.engage();
    }*/

}