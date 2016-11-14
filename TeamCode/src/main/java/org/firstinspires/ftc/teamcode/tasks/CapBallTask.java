package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends Thread {

    private DcMotor lift;
    private LinearOpMode opMode;
    public volatile boolean running = true;


    public CapBallTask(LinearOpMode opMode, DcMotor lift) {
        this.lift = lift;
        this.opMode = opMode;

    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if (opMode.gamepad2.right_bumper) {
                lift.setPower(1);
            }
            else if (opMode.gamepad2.left_bumper){
                lift.setPower(-1);
            }
            else {
                lift.setPower(0);
            }
        }
    }
}