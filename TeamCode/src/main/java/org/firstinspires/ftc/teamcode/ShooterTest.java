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

        motorA = hardwareMap.dcMotor.get("motorA");
        motorB = hardwareMap.dcMotor.get("motorB");
        motorA.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            motorA.setPower(-gamepad1.left_stick_y);
            motorB.setPower(motorA.getPower());
        }
    }
}
