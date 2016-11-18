package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;

/**
 * Created by bunnycide on 11/17/16.
 */
@TeleOp(name = "Lift Test", group = "Test")

public class LiftTest extends LinearOpMode {
    DcMotor capRight, capLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        capRight = hardwareMap.dcMotor.get("capRight");
        capLeft = hardwareMap.dcMotor.get("capLeft");
        capRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad2.right_bumper) {
                capRight.setPower(1);
                capLeft.setPower(capRight.getPower());
            }
            else if(gamepad2.left_bumper) {
                capRight.setPower(-1);
                capLeft.setPower(capRight.getPower());
            }
            else {
                capRight.setPower(0);
                capLeft.setPower(capRight.getPower());
            }
        }
    }
}
