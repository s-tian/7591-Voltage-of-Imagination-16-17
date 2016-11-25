package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.DriveTrainTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by bunnycide on 11/4/16.
 */

@TeleOp(name="Threaded Teleop", group = "Drive")

public class ThreadedTeleOp extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, conveyor, sweeper, capLeft, capRight;
    Servo gate, button, forkLeft, forkRight;
    public DriveTrainTask driveTrainTask;
    public FlywheelTask flywheelTask;
    public CapBallTask capBallTask;
    public IntakeTask intakeTask;
    //VOIImu imu;

    //ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {

        initialize();

        driveTrainTask = new DriveTrainTask(this, frontLeft, frontRight, backLeft, backRight);
        //ButtonPusherTask buttonPusherTask = new ButtonPusherTask(this, button);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        //IntakeTask intakeTask = new IntakeTask(this, sweeper, conveyor);
        capBallTask = new CapBallTask(this, capLeft, capRight, forkLeft, forkRight);
        waitForStart();
        long startTime = System.nanoTime();

        driveTrainTask.start();
        //buttonPusherTask.start();
        flywheelTask.start();
        //intakeTask.start();
        capBallTask.start();
        //driveTrainTask.zeroAngle = imu.getRadians();

        while(opModeIsActive()) {
            //Timer for 2 minute teleop period
            long elapsed = System.nanoTime() - startTime;
            //driveTrainTask.gyroAngle = imu.getRadians();

            if (elapsed > 120 * 1000000000L) {
                //Stop all tasks, the tasks will stop motors etc.
                driveTrainTask.running = false;
                //buttonPusherTask.running = false;
                flywheelTask.running = false;
                //intakeTask.running = false;
                capBallTask.running = false;
                //Get out of the loop
                break;
            } else {
                //telemetry.addData("Time elapsed", (int) (elapsed / 1000000000L));
                telemetry.addData("Flywheel status", flywheelTask.getFlywheelStateString());
            }
            telemetry.update();
        }
    }

    public void initialize(){
        capLeft = hardwareMap.dcMotor.get("capLeft");
        capRight = hardwareMap.dcMotor.get("capRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        //conveyor = hardwareMap.dcMotor.get("conveyor");
        //sweeper = hardwareMap.dcMotor.get("sweeper");
        //gate = hardwareMap.servo.get("gate");
        //button = hardwareMap.servo.get("button");
        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        //lift = hardwareMap.dcMotor.get("lift");
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu = new VOIImu(adaImu);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        forkLeft.setPosition(0.8);
        forkRight.setPosition(0.12);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        
        //gyro.calibrate();
        //gyro.resetZAxisIntegrator();
        //int base = gyro.getIntegratedZValue();
        //gate.setPosition(0.4);

    }
}
