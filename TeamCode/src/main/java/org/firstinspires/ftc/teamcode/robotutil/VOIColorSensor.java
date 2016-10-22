package org.firstinspires.ftc.teamcode.robotutil;

/**
 * Created by Stephen on 10/4/2016.
 */import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * VOIColorSensorFront.java
 * Class created to encapsulate all Color Sensor
 * functionality and output. Includes averaging multiple readings
 * for accurate output.
 *
 * Created by Stephen on 10/30/2015.
 */

public class VOIColorSensor {

    ColorSensor sensor;
    private ElapsedTime logMessageTimer;
    private String previousLogMessage;


    public VOIColorSensor(ColorSensor sensor)
    {
        this.sensor = sensor;
        logMessageTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        logMessageTimer.reset();
    }

    public void enableLED(boolean b)
    {
        sensor.enableLed(b);
    }

    public int getRed()
    {
        return sensor.red();
    }

    public int getBlue()
    {
        return sensor.blue();
    }

    public int getGreen()
    {
        return sensor.green();
    }

    public int getAlpha()
    {
        return sensor.alpha();
    }

    public void debugOutput(String output) {
        if(logMessageTimer.time() > 50) {
            logMessageTimer.reset();
            System.out.println(output);
            previousLogMessage = output;
        }
    }

    public boolean isWhite() {
        int red =  getRed();
        int blue = getBlue();
        int green = getGreen();
        debugOutput("Red: " + getRed() + " Blue: " + getBlue() + " Green " + getGreen());

        return (red >= 5 && blue >= 5 && green >= 5);
    }

    public boolean isBlue() {
        /* Averages multiple values for better reliability. */
        if (sensor.blue() >3)
            return true;
        return false;
    }

    public boolean isRed() {
        if (sensor.red() > 3)
            return true;
        return false;
    }



}