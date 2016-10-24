package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

/**
 * Created by Howard on 10/21/16.
 */
@TeleOp(name = "Color Test", group = "Tests")

public class ColorTest extends LinearOpMode {
    static final int topSensorID = 0x44;
    static final int bottomSensorID = 0x3c;
    VOIColorSensor voiTop, voiBottom;
    ColorSensor colorTop, colorBottom;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();


        while(opModeIsActive()) {
            telemetry.addData("Top:", "Red: " + voiTop.getRed() + " Blue: " + voiTop.getBlue() + " Green " + voiTop.getGreen());
            telemetry.addData("Bottom: ", "Red: " + voiBottom.getRed() + " Blue: " + voiBottom.getBlue() + " Green " + voiBottom.getGreen());
            telemetry.addData("Blue: ", voiTop.isBlue());
            telemetry.addData("Red: ", voiTop.isRed());
            telemetry.update();
        }
    }
    public void initialize(){
        colorTop = hardwareMap.colorSensor.get("colorTop");
        colorBottom = hardwareMap.colorSensor.get("colorBottom");
        colorTop.setI2cAddress(I2cAddr.create8bit(topSensorID));
        colorBottom.setI2cAddress(I2cAddr.create8bit(bottomSensorID));
        voiTop = new VOIColorSensor(colorTop, this);
        voiBottom = new VOIColorSensor(colorBottom, this);
    }
}