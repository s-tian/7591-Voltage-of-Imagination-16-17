package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.DriveTrainTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by bunnycide on 11/4/16.
 */

//@TeleOp(name="Constant Direction", group = "Drive")

public class ConstantDirection extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, conveyor, sweeper;
    Servo gate, button;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {

        initialize();

        DriveTrainTask driveTrainTask = new DriveTrainTask(this, frontLeft, frontRight, backLeft, backRight);
        ButtonPusherTask buttonPusherTask = new ButtonPusherTask(this, button);
        FlywheelTask flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        IntakeTask intakeTask = new IntakeTask(this, sweeper, conveyor);

        waitForStart();
        long startTime = System.nanoTime();

        driveTrainTask.start();
        buttonPusherTask.start();
        flywheelTask.start();
        intakeTask.start();

        while(opModeIsActive()) {
            //Timer for 2 minute teleop period
            long elapsed = System.nanoTime() - startTime;
            DriveTrainTask.gyroAngle = gyro.getIntegratedZValue();
            telemetry.addData("gyro" , DriveTrainTask.gyroAngle);
            telemetry.addData("Zero",DriveTrainTask.zeroAngle);
            telemetry.addData("Joystick",  DriveTrainTask.joyStickAngle);
            telemetry.addData("Total", DriveTrainTask.zeroAngle - DriveTrainTask.gyroAngle + DriveTrainTask.joyStickAngle);

            if (elapsed > 120 * 1000000000L) {
                //Stop all tasks, the tasks will stop motors etc.
                driveTrainTask.running = false;
                buttonPusherTask.running = false;
                flywheelTask.running = false;
                intakeTask.running = false;
                DriveTrainTask.gyroAngle = gyro.getIntegratedZValue();
                //Get out of the loop
                break;
            } else {
                //telemetry.addData("time elapsed", (int) (elapsed / 1000000000L));
            }
            if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                DriveTrainTask.zeroAngle = gyro.getIntegratedZValue();
            }

            //telemetry.update();
        }

    }
    public void initialize(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        gate = hardwareMap.servo.get("gate");
        button = hardwareMap.servo.get("button");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();
        gyro.resetZAxisIntegrator();
        int base = gyro.getIntegratedZValue();
        gate.setPosition(0.4);

    }
}

