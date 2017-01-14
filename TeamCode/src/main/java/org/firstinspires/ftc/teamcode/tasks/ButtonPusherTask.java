package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cController;
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
    Servo guide;
    public volatile boolean running = true;
    double power = 0;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int pushTime = 600;
    public int outTime = 2200;
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
        while (timer.time() < outTime - 300 && opMode.opModeIsActive());
        button.setPower(zeroPower);

    }
    //page 1117, 1132 formulas
    public void initialize() {

        button = opMode.hardwareMap.crservo.get("button");
        guide = opMode.hardwareMap.servo.get("guide");

        double mc7 = opMode.hardwareMap.voltageSensor.get("frontDrive").getVoltage();
        double mc6 = opMode.hardwareMap.voltageSensor.get("backDrive").getVoltage();
        double mc3 = opMode.hardwareMap.voltageSensor.get("cap").getVoltage();
        double mc2 = opMode.hardwareMap.voltageSensor.get("flywheels").getVoltage();
        double voltageLevel = (mc7 + mc6 + mc3 + mc2) / 4;
        pushTime *= 13.0/voltageLevel;
        outTime *= 13.0/voltageLevel;

        button.setPower(0);
    }

}
