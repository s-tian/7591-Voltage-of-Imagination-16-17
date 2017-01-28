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
    public int pushTime = 650;
    public int outTime = 1900;
    public static final double zeroPower = 0;
    public static final double outPower = -1;
    public static final double inPower = 1;
    public static final double upPosition = 1;
    public static final double downPosition = 0.2;
    public volatile boolean teleOp = false;
    public boolean guideDown = false;
    public boolean guidePushed = false;
    private boolean pushButton = false;
    private boolean extendButton = false;
    private boolean withdrawButton = false;

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
                buttonPush();
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

    private void buttonPush() {
        button.setPower(outPower);
        sleep(pushTime);
        button.setPower(inPower);
        pushTime -= 100;
        sleep(pushTime);
        button.setPower(zeroPower);
    }

    private void outPusher() {
        timer.reset();
        button.setPower(outPower);
        sleep(outTime);
        button.setPower(zeroPower);
    }

    private void inPusher() {
        timer.reset();
        button.setPower(inPower);
        sleep(outTime);
        button.setPower(zeroPower);

    }

    public void out() {
        extendButton = true;
    }

    public void in() {
        // TODO: 1/25/17 not all the way in 
        withdrawButton = true;
    }

    public void push() {
        pushButton = true;
    }



}
