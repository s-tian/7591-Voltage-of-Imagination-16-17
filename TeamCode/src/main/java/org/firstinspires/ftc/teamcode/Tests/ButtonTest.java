package org.firstinspires.ftc.teamcode.Tests;

import android.widget.Button;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/8/17.
 */

@TeleOp(name = "Button Test", group = "Test")


public class ButtonTest extends LinearOpMode {

    CRServo button;
    ButtonPusherTask buttonPusherTask;
    CapBallTask capBallTask;
    DcMotor capBottom;
    static int pushTime = 500;
    Servo guide;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        choosePower();
        waitForStart();
        buttonPusherTask.start();
        buttonPusherTask.out();
        sleep(2000);
    }
    public void initialize() {
        button = hardwareMap.crservo.get("button");
        button.setPower(ButtonPusherTask.zeroPower);
        capBallTask = new CapBallTask(this);
        new IntakeTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
        capBottom = hardwareMap.dcMotor.get("capBottom");
    }
    public void choosePower() {
        boolean upPressed = false, downPressed = false;
        boolean confirmed = false;
        while (!confirmed) {
            if (gamepad1.dpad_up && !upPressed) {
                pushTime += 50;
                upPressed = true;
            }
            if (gamepad1.dpad_down && !downPressed) {
                pushTime -= 50;
                downPressed = true;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            telemetry.addData("pushTime", pushTime);
            telemetry.update();
            confirmed = gamepad1.left_stick_button && gamepad1.right_stick_button;
        }
    }
}