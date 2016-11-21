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
    DcMotor capRight, capLeft, frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        capRight = hardwareMap.dcMotor.get("capRight");
        capLeft = hardwareMap.dcMotor.get("capLeft");
        capRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                capRight.setPower(-1);
                capLeft.setPower(capRight.getPower());
            } else if (gamepad1.left_bumper) {
                capRight.setPower(1);
                capLeft.setPower(capRight.getPower());
            } else {
                capRight.setPower(0);
                capLeft.setPower(capRight.getPower());
            }

            double joy1Y = -gamepad1.left_stick_y/2;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y * 3 / 4 : 0;
            double  joy1X = gamepad1.left_stick_x/2;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X * 3 / 4 : 0;
            double joy2X = gamepad1.right_stick_x/2;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X * 3 / 4 : 0;
            frontLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X + joy1X)));
            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));
        }
    }
}