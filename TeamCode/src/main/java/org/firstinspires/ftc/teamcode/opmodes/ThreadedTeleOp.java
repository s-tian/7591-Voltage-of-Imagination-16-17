package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.DriveTrainTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

import java.text.DecimalFormat;

/**
 * Created by bunnycide on 11/4/16.
 */

@TeleOp(name="Threaded Teleop", group = "Drive")

public class ThreadedTeleOp extends LinearOpMode {

    Servo guide;
    public DriveTrainTask driveTrainTask;
    public FlywheelTask flywheelTask;
    public CapBallTask capBallTask;
    public IntakeTask intakeTask;
    public ButtonPusherTask buttonPusherTask;
    public double voltageLevel;

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
        //driveTrainTask.zeroAngle = imu.getRadians();

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        while(opModeIsActive()) {
            //Timer for 2 minute teleop period
            long elapsed = System.nanoTime() - startTime;
            //driveTrainTask.gyroAngle = imu.getRadians();

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

                telemetry.addData("Time elapsed", (int) (elapsed / 1000000000L));
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft*100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight*100));
                telemetry.addData("Flywheel state", flywheelTask.getFlywheelState());
            }
            telemetry.update();
        }
    }

    public void initialize(){
        guide = hardwareMap.servo.get("guide");
        guide.setPosition(0.9);
        double mc7 = hardwareMap.voltageSensor.get("frontDrive").getVoltage();
        double mc6 = hardwareMap.voltageSensor.get("backDrive").getVoltage();
        double mc3 = hardwareMap.voltageSensor.get("cap").getVoltage();
        double mc2 = hardwareMap.voltageSensor.get("flywheels").getVoltage();
        voltageLevel = (mc7 + mc6 + mc3 + mc2) / 4;
        driveTrainTask = new DriveTrainTask(this);
        flywheelTask = new FlywheelTask(this);
        flywheelTask.voltage = voltageLevel;
        intakeTask = new IntakeTask(this);
        capBallTask = new CapBallTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
        buttonPusherTask.teleOp = intakeTask.teleOp = flywheelTask.teleOp = true;
    }
}
