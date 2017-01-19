package org.firstinspires.ftc.teamcode.calibration;

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

public class CapPowerCalibration extends LinearOpMode {
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

            if (gamepad1.a && !aPressed) {
                aPressed = true;
                CapBallTask.holdPower += 0.001;
            }
            if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.b && !bPressed) {
                bPressed = true;
                CapBallTask.holdPower -= 0.001;
            }
            if (!gamepad1.b) {
                bPressed = false;
            }
            if (gamepad1.x && !xPressed) {
                xPressed = true;
                CapBallTask.editPower += 0.0001;
            }
            if (!gamepad1.x) {
                xPressed = false;
            }
            if (gamepad1.y && !yPressed) {
                yPressed = true;
                CapBallTask.editPower -= 0.0001;
            }
            if (!gamepad1.y) {
                yPressed = false;
            }
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
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
