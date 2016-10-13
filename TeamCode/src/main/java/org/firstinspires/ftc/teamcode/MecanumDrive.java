package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Mecanum Drive", group = "Drive")

public class MecanumDrive extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, motorA, motorB, motorC;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean increased = false ,decreased = false;
        boolean cIncreased = false, cDecreased = false;

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        motorA = hardwareMap.dcMotor.get("motorA");
        motorB = hardwareMap.dcMotor.get("motorB");
        motorC = hardwareMap.dcMotor.get("motorC");

        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorA.setDirection(DcMotorSimple.Direction.REVERSE);
        motorC.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Flywheel", motorA.getPower());
            telemetry.addData("Conveyor", motorC.getPower());

            double joy1Y = -gamepad1.left_stick_y;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
            double joy1X = gamepad1.left_stick_x;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
            double joy2X = gamepad1.right_stick_x;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
            frontLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X + joy1X)));
            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));

            if (gamepad1.a && motorA.getPower() <= .9 && !increased) {
                motorA.setPower(motorA.getPower() + .1);
                motorB.setPower(motorA.getPower());
                increased = true;
            }
            if (gamepad1.b && motorA.getPower() >= .1 && !decreased) {
                motorA.setPower(motorA.getPower() - .1);
                motorB.setPower(motorA.getPower());
                decreased = true;
            }
            if (gamepad1.x && motorC.getPower() <= .9 && !cIncreased){
                motorC.setPower(motorC.getPower() + .1);
                cIncreased = true;
            }
            if (gamepad1.y && motorC.getPower() >= .1 && !cDecreased){
                motorC.setPower(motorC.getPower()- .1);
                cDecreased = true;
            }
            if (!gamepad1.a){
                increased = false;
            }
            if (!gamepad1.b) {
                decreased = false;
            }
            if (!gamepad1.x){
                cIncreased = false;
            }
            if (!gamepad1.y){
                cDecreased = false;
            }

            telemetry.update();

        }
    }
}
