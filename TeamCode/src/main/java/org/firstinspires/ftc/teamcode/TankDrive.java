package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bunnycide on 10/5/16.
 */

@TeleOp(name = "Tank Drive", group = "Drive")

public class TankDrive extends LinearOpMode {

    DcMotor backLeft, backRight, frontLeft, frontRight, midfrontLeft, midfrontRight, midbackLeft, midbackRight;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("left1");
        frontRight = hardwareMap.dcMotor.get("right1");
        midfrontLeft = hardwareMap.dcMotor.get("left2");
        midfrontRight = hardwareMap.dcMotor.get("right2");
        midbackLeft = hardwareMap.dcMotor.get("left3");
        midbackRight = hardwareMap.dcMotor.get("right3");
        backLeft = hardwareMap.dcMotor.get("left4");
        backRight = hardwareMap.dcMotor.get("right4");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midbackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midbackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midfrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        midfrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        midbackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        midfrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            double joy1Y = gamepad1.left_stick_y;
            double joy2Y = gamepad1.right_stick_y;
            backLeft.setPower(joy1Y);
            backRight.setPower(joy2Y);
            midbackLeft.setPower(joy1Y);
            midbackRight.setPower(joy2Y);
            midfrontLeft.setPower(joy1Y);
            midfrontRight.setPower(joy2Y);
            frontLeft.setPower(joy1Y);
            frontRight.setPower(joy2Y);
        }
    }
}
