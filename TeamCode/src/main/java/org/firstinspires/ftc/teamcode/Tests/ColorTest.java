package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

/**
 * Created by Howard on 10/21/16.
 */
@TeleOp(name = "Color Test", group = "Test")

public class ColorTest extends LinearOpMode {
    static final int topSensorID = 0x3a;
    static final int bottomBackID = 0x3c;
    static final int bottomFrontID = 0x44;
    VOIColorSensor voiTop, voiBottomBack, voiBottomFront;
    ColorSensor colorTop, colorBottomBack, colorBottomFront;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();


        while(opModeIsActive()) {
            telemetry.addData("Top:", "Red: " + voiTop.getRed() + " Blue: " + voiTop.getBlue() + " Green " + voiTop.getGreen());
            telemetry.addData("BottomBack: ", "Red: " + voiBottomBack.getRed() + " Blue: " + voiBottomBack.getBlue() + " Green " + voiBottomBack.getGreen());
            telemetry.addData("BottomFront: ", "Red: " + voiBottomFront.getRed() + " Blue: " + voiBottomFront.getBlue() + " Green " + voiBottomFront.getGreen());
            telemetry.addData("Blue: ", voiTop.isBlue());
            telemetry.addData("Red: ", voiTop.isRed());
            telemetry.update();
        }
    }
    public void initialize(){
        colorTop = hardwareMap.colorSensor.get("colorTop");
        colorBottomBack = hardwareMap.colorSensor.get("colorBottomBack");
        colorBottomFront = hardwareMap.colorSensor.get("colorBottomFront");
        colorTop.setI2cAddress(I2cAddr.create8bit(topSensorID));
        colorBottomBack.setI2cAddress(I2cAddr.create8bit(bottomBackID));
        colorBottomFront.setI2cAddress(I2cAddr.create8bit(bottomFrontID));

        voiTop = new VOIColorSensor(colorTop, this);
        voiBottomBack = new VOIColorSensor(colorBottomBack, this);
        voiBottomFront = new VOIColorSensor(colorBottomFront, this);
    }
}