package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

import java.text.DecimalFormat;

/**
 * Created by Howard on 1/17/17.
 */
@TeleOp(name = "Cap power", group = "Test")

public class CapPower extends LinearOpMode {
    CapBallTask capBallTask;
    IntakeTask intakeTask;
    ButtonPusherTask buttonPusherTask;
    DecimalFormat df = new DecimalFormat();

    public void runOpMode() {
        df.setMaximumFractionDigits(5);
        initialize();
        changePower();
        waitForStart();
        capBallTask.start();
        while (opModeIsActive());
    }
    public void initialize() {
        capBallTask = new CapBallTask(this);
        intakeTask = new IntakeTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
    }

    public void changePower () {
        boolean confirmed = false;
        boolean aPressed = false, bPressed = false, xPressed = false, yPressed = false;
        while (!confirmed) {

            if (gamepad2.a && !aPressed) {
                aPressed = true;
                CapBallTask.holdPower += 0.01;
            }
            if (!gamepad2.a) {
                aPressed = false;
            }
            if (gamepad2.b && !bPressed) {
                bPressed = true;
                CapBallTask.holdPower -= 0.01;
            }
            if (!gamepad2.b) {
                bPressed = false;
            }
            if (gamepad2.x && !xPressed) {
                xPressed = true;
                CapBallTask.editPower += 0.01;
            }
            if (!gamepad2.x) {
                xPressed = false;
            }
            if (gamepad2.y && !yPressed) {
                yPressed = true;
                CapBallTask.editPower -= 0.01;
            }
            if (!gamepad2.y) {
                yPressed = false;
            }
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
            }
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
            }
            telemetry.addData("holdPower", df.format(CapBallTask.holdPower));
            telemetry.addData("editPower", df.format(CapBallTask.editPower));
            telemetry.update();
        }
        telemetry.addData("holdPower", df.format(CapBallTask.holdPower));
        telemetry.addData("editPower", df.format(CapBallTask.editPower));
        telemetry.addData("Confirmed!", "");
        telemetry.update();

    }

}
