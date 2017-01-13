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
    double upPosition = 0.7;
    double downPosition = 0.1;
    public boolean teleOp = false;
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
                if (Math.abs(opMode.gamepad2.left_stick_y) > 0.15) {
                    button.setPower(opMode.gamepad2.left_stick_y);
                }
                if (opMode.gamepad2.left_bumper) {
                    guide.setPosition(downPosition);
                } else {
                    guide.setPosition(upPosition);
                }
            }
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
        while (timer.time() < outTime - 200 && opMode.opModeIsActive());
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
