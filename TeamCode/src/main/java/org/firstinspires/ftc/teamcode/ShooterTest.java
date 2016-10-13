package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Shooter Test", group = "Test")

public class ShooterTest extends LinearOpMode {

    DcMotor motorA, motorB, motorC;


    @Override
    public void runOpMode() throws InterruptedException {
        boolean increased = false ,decreased = false;
        boolean cIncreased = false, cDecreased = false;
        motorA = hardwareMap.dcMotor.get("motorA");
        motorB = hardwareMap.dcMotor.get("motorB");
        motorC = hardwareMap.dcMotor.get("motorC");
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorA.setDirection(DcMotorSimple.Direction.REVERSE);
        motorC.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Flywheel", motorA.getPower());
            telemetry.addData("Conveyor", motorC.getPower());
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
