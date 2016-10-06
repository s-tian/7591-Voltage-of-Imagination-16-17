package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bunnycide on 10/5/16.
 */
@TeleOp(name = "CR Test", group = "Drive")

public class CRTest extends LinearOpMode{

    CRServo servo1;
    CRServo servo2;

    @Override
    public void runOpMode() throws InterruptedException{
        servo1 = hardwareMap.crservo.get("cr1");
        servo2 = hardwareMap.crservo.get("cr2");

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                servo1.setPower(1);
                servo2.setPower(1);
            } else if (gamepad1.b) {
                servo1.setPower(-1);
                servo2.setPower(-1);
            } else {
                servo1.setPower(0);
                servo2.setPower(0);
            }
        }
    }
}
