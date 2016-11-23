package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.R.layout.servo;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends Thread {

    private DcMotor capLeft, capRight;
    private Servo forkLeft, forkRight;
    private LinearOpMode opMode;
    boolean down = false;
    double startLeft = 0.8, startRight = 0.12, downLeft = 0.3, downRight = 0.62;
    boolean aPushed = false;
    public volatile boolean running = true;


    public CapBallTask(LinearOpMode opMode, DcMotor capLeft, DcMotor capRight, Servo forkLeft, Servo forkRight) {
        this.capLeft = capLeft;
        this.capRight = capRight;
        this.forkLeft = forkLeft;
        this.forkRight = forkRight;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if (opMode.gamepad1.right_bumper) {
                setCapPower(1);
            }
            else if (opMode.gamepad1.left_bumper){
                setCapPower(-1);
            }
            else {
                setCapPower(0);
            }
            if (opMode.gamepad1.a || opMode.gamepad2.a && !aPushed) {
                if (down){
                    forkLeft.setPosition(startLeft);
                    forkRight.setPosition(startRight);
                    down = false;
                } else {
                    forkLeft.setPosition(downLeft);
                    forkRight.setPosition(downRight);
                    down = true;
                }
                aPushed = true;
            }
            if (!opMode.gamepad1.a && !opMode.gamepad2.a){
                aPushed = false;
            }
        }
    }

    public void setCapPower(double power){
        capLeft.setPower(power);
        capRight.setPower(power);
    }

}