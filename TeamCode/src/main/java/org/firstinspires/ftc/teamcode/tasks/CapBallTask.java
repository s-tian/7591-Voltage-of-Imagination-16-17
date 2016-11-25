package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;

import static org.firstinspires.ftc.teamcode.R.layout.servo;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends Thread {

    private DcMotor capLeft, capRight;
    private Servo forkLeft, forkRight;
    private ThreadedTeleOp opMode;
    private ElapsedTime timer  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double startLeft = 0.8, startRight = 0.12, downLeft = 0.3, downRight = 0.62;
    boolean aPushed = false;
    public volatile boolean running = true;
    boolean slideOut = true;
    boolean slideRunning = false;
    boolean ballUp = false;
    boolean slideIn = false;


    public CapBallTask(ThreadedTeleOp opMode, DcMotor capLeft, DcMotor capRight, Servo forkLeft, Servo forkRight) {

        this.capLeft = capLeft;
        this.capRight = capRight;
        this.forkLeft = forkLeft;
        this.forkRight = forkRight;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {
            if (opMode.gamepad1.right_bumper||opMode.gamepad2.right_trigger-opMode.gamepad2.left_trigger>0.15) {
                setCapPower(1);
                slideRunning = true;
            }
            else if (opMode.gamepad1.left_bumper||opMode.gamepad2.left_trigger-opMode.gamepad2.right_trigger>0.15){
                setCapPower(-1);
                slideRunning = true;
            }
            else {
                setCapPower(0);
                slideRunning = false;
            }
            /*
            if (opMode.gamepad1.dpad_down || opMode.gamepad2.a && !aPushed) {
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
            if (!opMode.gamepad1.dpad_down && !opMode.gamepad2.a){
                aPushed = false;
            }*/
            if(opMode.gamepad2.right_bumper) {
                forkLeft.setPosition(downLeft);
                forkRight.setPosition(downRight);
                slideIn = true;
            }
            else {
                forkLeft.setPosition(startLeft);
                forkRight.setPosition(startRight);
            }
            if(opMode.gamepad1.dpad_down) {
                timer.reset();
                slideOut = false;
                while(timer.time()<500 && !slideOut) {
                    setCapPower(1);
                }
                while(timer.time()>500 && timer.time()<1400 && !slideOut) {
                    setCapPower(-1);
                }
                slideOut = true;
                ballUp = true;
            }
        }
    }

    public void setCapPower(double power){
        capLeft.setPower(power);
        capRight.setPower(power);
    }
    public boolean isBallUp() {
        return slideRunning && ballUp;
    }
    public void setBallUp(boolean ballUp) {
        this.ballUp = ballUp;
    }
    public boolean isSlideIn() {
        return slideIn;
    }
    public void setSlideIn(boolean slideIn) {
        this.slideIn = slideIn;
    }
}