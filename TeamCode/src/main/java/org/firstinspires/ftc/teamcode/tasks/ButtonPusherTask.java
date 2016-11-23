package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class ButtonPusherTask extends Thread {

    private Servo servo;
    private LinearOpMode opMode;
    private boolean pressing = false;
    private boolean buttonOut = false;
    private boolean xPushed = false;
    private boolean yPushed = false;
    public volatile boolean running = true;

/* initailize servos upwards
encoder on slide motors
make it automated
get out forklift driver 1
1. raise slides
2. lower slides
Open servo, 5 secs, close servo
  */
    public ButtonPusherTask(LinearOpMode opMode, Servo servo) {
        this.servo = servo;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opMode.opModeIsActive() && running) {
            if (opMode.gamepad1.x) {
                if (!xPushed && !buttonOut) {
                    buttonOut = true;
                    servo.setPosition(1);
                    timer.reset();
                    pressing = true;
                    xPushed = true;
                }
            } else {
                xPushed = false;
                buttonOut = false;
                servo.setPosition(0);
            }
            if(opMode.gamepad2.y) {
                servo.setPosition(1);
                buttonOut = true;
                pressing = false;
                yPushed = true;
            } else if (yPushed) {
                servo.setPosition(0);
                buttonOut = false;
                yPushed = false;
            }

        }
    }
}
