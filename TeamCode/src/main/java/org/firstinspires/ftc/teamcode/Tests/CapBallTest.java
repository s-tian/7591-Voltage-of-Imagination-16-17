package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Howard on 11/17/16.
 */

@TeleOp(name = "Cap Ball Test", group = "Test")

public class CapBallTest extends LinearOpMode {

    DcMotor capLeft;
    DcMotor capRight;

    @Override
    public void runOpMode() throws InterruptedException{
        capLeft = hardwareMap.dcMotor.get("capLeft");
        capRight = hardwareMap.dcMotor.get("capRight");
        capLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad2.right_bumper){
                setCapPower(1);
            }
            else if (gamepad2.left_bumper){
                setCapPower(-1);
            }
            else {
                setCapPower(0);
            }
        }
    }
    public void setCapPower(double power){
        capLeft.setPower(power);
        capRight.setPower(power);
    }
}
