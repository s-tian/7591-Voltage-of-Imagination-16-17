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
    static final int backID = 0x3a;
    static final int frontID = 0x3c;
    VOIColorSensor voiFront, voiBack;
    ColorSensor colorFront, colorBack;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Back:", "Red: " + voiBack.getRed() + " Blue: " + voiBack.getBlue() + " Green " + voiBack.getGreen());
            telemetry.addData("Front: ", "Red: " + voiFront.getRed() + " Blue: " + voiFront.getBlue() + " Green " + voiFront.getGreen());
            if (voiBack.isBlue()) {
                telemetry.addData("Blue", "Back");
            } else if (voiFront.isBlue()) {
                telemetry.addData("Blue", "Front");
            } else {
                telemetry.addData("Blue", "None");
            }
            if (voiBack.isRed()) {
                telemetry.addData("Red", "Back");
            } else if (voiFront.isRed()) {
                telemetry.addData("Red", "Front");
            } else {
                telemetry.addData("Red", "None");
            }
            telemetry.update();
        }
    }
    public void initialize(){
        colorBack = hardwareMap.colorSensor.get("colorBack");
        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorBack.setI2cAddress(I2cAddr.create8bit(backID));
        colorFront.setI2cAddress(I2cAddr.create8bit(frontID));

        voiBack = new VOIColorSensor(colorBack, this);
        voiFront = new VOIColorSensor(colorFront, this);
        voiFront.weak = true;
    }
}