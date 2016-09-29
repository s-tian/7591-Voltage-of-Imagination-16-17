package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Shooter Test", group = "Test")

public class ShooterTest extends LinearOpMode {

    DcMotor motorA, motorB;


    @Override
    public void runOpMode() throws InterruptedException {
        boolean increased = false ,decreased = false;

        motorA = hardwareMap.dcMotor.get("motorA");
        motorB = hardwareMap.dcMotor.get("motorB");
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a && motorA.getPower() <= .9 && !increased){
                motorA.setPower(motorA.getPower()+.1);
                motorB.setPower(motorA.getPower());
                increased = true;
            }
            if (gamepad1.b && motorA.getPower() >= .1 && !decreased) {
                motorA.setPower(motorA.getPower() - .1);
                motorB.setPower(motorA.getPower());
                decreased = true;
            }
            if (!gamepad1.a){
                increased = false;
            }
            if (!gamepad1.b) {
                decreased = false;
            }

        }
    }
}
