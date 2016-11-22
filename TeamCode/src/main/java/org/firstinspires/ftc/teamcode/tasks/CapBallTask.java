package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends Thread {

    private DcMotor capRight, capLeft;
    private LinearOpMode opMode;
    public volatile boolean running = true;


    public CapBallTask(LinearOpMode opMode, DcMotor capRight, DcMotor capLeft) {
        this.capRight = capRight;
        this.capLeft = capLeft;
        this.opMode = opMode;

    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if (opMode.gamepad1.right_bumper) {
                capRight.setPower(1);
                capLeft.setPower(capRight.getPower());
            }
            else if (opMode.gamepad1.left_bumper) {
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