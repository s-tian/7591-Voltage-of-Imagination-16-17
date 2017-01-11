package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tests.ButtonTest;
import org.firstinspires.ftc.teamcode.opmodes.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;

/**
 * Created by Howard on 10/15/16.
 */
public class ButtonPusherTask extends Thread {

    private LinearOpMode opMode;
    CRServo button;
    public volatile boolean running = true;
    double power = 0;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static final int pushTime = 800;
    public static final int outTime = 2200;
    public static final double zeroPower = 0;
    public static final double outPower = -1;
    public static final double inPower = 1;
    public boolean pushButton = false;
    public boolean extendButton = false;
    public boolean withdrawButton = false;

    public ButtonPusherTask(LinearOpMode opMode, CRServo pusher) {
        this.button = pusher;
        this.opMode = opMode;
        pusher.setPower(zeroPower);
    }

    @Override
    public void run() {
        boolean rPushed = false, lPushed = false;
        while(opMode.opModeIsActive() && running) {
            if (pushButton) {
                pushButton = false;
                pushButton();
            }
            else if (extendButton) {
                extendButton = false;
                outPusher();
            }
            else if (withdrawButton) {
                withdrawButton = false;
                inPusher();
            }


            if (opMode.gamepad2.right_bumper && !rPushed) {
                //pusher.setPower(1);
                power += 1;
                rPushed = true;
            } else if(opMode.gamepad2.left_bumper && !lPushed) {
                //pusher.setPower(-1);
                power -= 1;
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
            //button.setPower(power);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();
        }
        button.setPower(0);

    }

    public void pushButton() {
        pushOut();
        timer.reset();
        button.setPower(inPower);
        while (timer.time() < pushTime && opMode.opModeIsActive());
        button.setPower(zeroPower);
    }
    public void pushOut() {
        button.setPower(outPower);
        timer.reset();
        while (timer.time() < pushTime && opMode.opModeIsActive());
    }
    public void outPusher() {
        timer.reset();
        button.setPower(outPower);
        while (timer.time() < outTime && opMode.opModeIsActive());
        button.setPower(zeroPower);
    }

    public void inPusher() {
        timer.reset();
        button.setPower(inPower);
        while (timer.time() < outTime - 100 && opMode.opModeIsActive());
        button.setPower(zeroPower);

    }

}
