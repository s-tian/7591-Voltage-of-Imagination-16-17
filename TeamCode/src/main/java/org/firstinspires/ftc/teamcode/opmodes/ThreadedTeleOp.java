package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.DriveTrainTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;

import java.text.DecimalFormat;

/**
 * Created by bunnycide on 11/4/16.
 * TeleOp
 */

@TeleOp(name="Threaded Teleop", group = "Drive")

public class ThreadedTeleOp extends LinearOpMode {

    private Servo guide;
    private DriveTrainTask driveTrainTask;
    private FlywheelTask flywheelTask;
    private CapBallTask capBallTask;
    private IntakeTask intakeTask;
    private ButtonPusherTask buttonPusherTask;

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        long startTime = System.nanoTime();

        driveTrainTask.start();
        flywheelTask.start();
        intakeTask.start();
        capBallTask.start();
        buttonPusherTask.start();

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        while(opModeIsActive()) {
            //Timer for 2 minute teleop period
            long elapsed = System.nanoTime() - startTime;

            if (elapsed > 120 * 1000000000L) {
                //Stop all tasks, the tasks will stop motors etc.
                driveTrainTask.running = false;
                buttonPusherTask.running = false;
                flywheelTask.running = false;
                intakeTask.running = false;
                capBallTask.running = false;
                //Get out of the loop
                break;
            } else {
                int seconds = 120 - (int) (elapsed/1000000000L);
                String timeString = (seconds/60) + ":";
                if (seconds%60 < 10) {
                    timeString += 0;
                }
                timeString += seconds%60;
                telemetry.addData("Time elapsed", timeString);
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft*100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight*100));
                telemetry.addData("Flywheel state", flywheelTask.state);
            }
            telemetry.update();
        }
    }

    public void initialize(){
        guide = hardwareMap.servo.get("guide");
        guide.setPosition(ButtonPusherTask.upPosition);
        driveTrainTask = new DriveTrainTask(this);
        flywheelTask = new FlywheelTask(this);
        intakeTask = new IntakeTask(this);
        capBallTask = new CapBallTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
        TaskThread.calculateVoltage(this);
        buttonPusherTask.teleOp = intakeTask.teleOp = flywheelTask.teleOp = true;
    }
}
