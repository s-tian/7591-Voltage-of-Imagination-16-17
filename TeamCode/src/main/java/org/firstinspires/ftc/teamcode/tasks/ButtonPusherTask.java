package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 10/15/16.
 */
public class ButtonPusherTask extends TaskThread {

    CRServo button;
    Servo guide;
    double power = 0;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int pushTime = 800;
    public int outTime = 2000;
    public static final double zeroPower = 0;
    public static final double outPower = -1;
    public static final double inPower = 1;
    public static final double upPosition = 1;
    public static final double downPosition = 0.2;
    public volatile boolean teleOp = false;
    public boolean guideDown = false;
    public boolean guidePushed = false;
    public volatile boolean pushButton = false;
    public volatile boolean extendButton = false;
    public volatile boolean withdrawButton = false;

    public ButtonPusherTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void initialize() {
        button = opMode.hardwareMap.crservo.get("button");
        guide = opMode.hardwareMap.servo.get("guide");
        pushTime *= EXPECTED_VOLTAGE / voltage;
        outTime *= EXPECTED_VOLTAGE / voltage;
        button.setPower(0);
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {

            // Autonomous
            if (pushButton) {
                pushButton = false;
                pushButton();
            } else if (extendButton) {
                extendButton = false;
                outPusher();
            } else if (withdrawButton) {
                withdrawButton = false;
                inPusher();
            }

            //TeleOp
            if (teleOp) {
                if (Math.abs(opMode.gamepad2.right_stick_x) > 0.15|| Math.abs(opMode.gamepad2.left_stick_x) > 0.15) {
                    button.setPower(-opMode.gamepad2.right_stick_x-opMode.gamepad2.left_stick_x);
                    guideDown();
                } else {
                    button.setPower(0);
                }
                if (opMode.gamepad2.left_bumper && !guideDown && !guidePushed) {
                    guideDown();
                    guidePushed = true;
                }
                else if (opMode.gamepad2.left_bumper && guideDown && !guidePushed){
                    guideUp();
                    guidePushed = true;
                }
                else if (!opMode.gamepad2.left_bumper){
                    guidePushed = false;
                }
            }
        }
        button.setPower(0);

    }

    public void guideDown() {
        guide.setPosition(downPosition);
        guideDown = true;
    }

    public void guideUp() {
        guide.setPosition(upPosition);
        guideDown = false;
    }

    public void pushButton() {
        button.setPower(outPower);
        sleep(pushTime);
        button.setPower(inPower);
        sleep(pushTime);
        button.setPower(zeroPower);
    }

    public void outPusher() {
        timer.reset();
        button.setPower(outPower);
        sleep(outTime);
        button.setPower(zeroPower);
    }

    public void inPusher() {
        timer.reset();
        button.setPower(inPower);
        sleep(outTime - 300);
        button.setPower(zeroPower);

    }



}
