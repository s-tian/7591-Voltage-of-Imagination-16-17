package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class ButtonPusherTask extends Thread {

    private ThreadedTeleOp opMode;
    CRServo pusher;
    public volatile boolean running = true;
    double power = 0;

/* initailize servos upwards
encoder on slide motors
make it automated
get out forklift driver 1
1. raise slides
2. lower slides
Open servo, 5 secs, close servo
  */
    public ButtonPusherTask(ThreadedTeleOp opMode, CRServo pusher) {
        this.pusher = pusher;
        this.opMode = opMode;
        pusher.setPower(0);

    }

    @Override
    public void run() {
        boolean rPushed = false, lPushed = false;
        while(opMode.opModeIsActive() && running) {

            if (opMode.gamepad2.right_bumper && !rPushed) {
                //pusher.setPower(1);
                power += 0.005;
                rPushed = true;
            } else if(opMode.gamepad2.left_bumper && !lPushed) {
                //pusher.setPower(-1);
                power -= 0.005;
                lPushed = true;
            } else {
                //pusher.setPower(0);
            }
            if (!opMode.gamepad2.right_bumper) {
                rPushed = false;
            }
            if (!opMode.gamepad2.left_bumper) {
                lPushed = false;
            }
            pusher.setPower(power);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

        }
    }

}
