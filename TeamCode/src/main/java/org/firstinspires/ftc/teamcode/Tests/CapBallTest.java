package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Howard on 11/17/16.
 */

@TeleOp(name = "Cap Ball Test", group = "Test")

public class CapBallTest extends LinearOpMode {

    DcMotor capLeft, capRight;
    Servo forkLeft, forkRight;

    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.right_bumper){
                setCapPower(1);
            }
            else if (gamepad1.left_bumper){
                setCapPower(-1);
            }
            else {
                setCapPower(0);
            }
            if (gamepad1.a){
                foldForkLift();
            }
        }
    }
    public void setCapPower(double power){
        capLeft.setPower(power);
        capRight.setPower(power);
    }
    public void foldForkLift(){
        forkLeft.setPosition(0.3);
        forkRight.setPosition(0.62);
        sleep(5000);
        forkLeft.setPosition(0.8);
        forkRight.setPosition(0.12);
    }
    public void initialize(){
        capLeft = hardwareMap.dcMotor.get("capLeft");
        capRight = hardwareMap.dcMotor.get("capRight");
        capLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        capLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capRight.setDirection(DcMotorSimple.Direction.REVERSE);
        forkLeft.setPosition(0.8);
        forkRight.setPosition(0.12);
    }
}
