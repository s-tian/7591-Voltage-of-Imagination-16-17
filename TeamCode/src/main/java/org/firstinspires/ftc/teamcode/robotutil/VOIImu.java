package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Howard on 11/9/16.
 */

public class VOIImu{
    public BNO055IMU adafruit;
    Orientation angles;
    Acceleration gravity;
    public VOIImu(BNO055IMU adafruit){
        this.adafruit = adafruit;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        adafruit.initialize(parameters);
    }
    public int getAngle(){
        angles = adafruit.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        int angle = (int)AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        return -angle;
    }
    public double getRadians(){
        return getAngle()*Math.PI/180;
    }

    public static int addAngles(int angle1, int angle2){
        int sum = (angle1 + angle2)%360;
        if (sum >= 180){
            sum -= 360;
        } else if (sum <= -180){
            sum += 360;
        }
        return sum;
    }

    public static int subtractAngles(double angle1, double angle2){
        if (angle1 < 0) {
            angle1 += 360;
        }
        if (angle2 < 0) {
            angle2 += 360;
        }
        int diff1 = (int)(angle1 - angle2);
        int diff2 = diff1 + 360;
        int diff3 = diff1 - 360;
        if (Math.abs(diff1) <= Math.abs(diff2) && Math.abs(diff1) <= Math.abs(diff3)){
            return diff1;
        } else if (Math.abs(diff2) < Math.abs(diff3)){
            return diff2;
        }
        System.out.println(diff1 + " " + diff2 + " " + diff3);
        return diff3;
    }

    public static int subtractAngles(double angle1, double angle2, boolean clockwise) {
        // angle1 - angle2
        if (angle1 < 0) {
            angle1 += 360;
        }
        if (angle2 < 0) {
            angle2 += 360;
        }
        if (clockwise && angle1 < angle2) {
            angle1 += 360;
        } else if (!clockwise && angle1 > angle2) {
            angle1 -= 360;
        }
        return (int)(angle1 - angle2);

    }

}
