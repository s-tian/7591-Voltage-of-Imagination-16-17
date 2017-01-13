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

    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, capBottom, capTop;
    CRServo sweeper1, sweeper2, sweeper3;
    VOISweeper sweeper;
    Servo forkLeft, forkRight, guide;
    CRServo button;
    public DriveTrainTask driveTrainTask;
    public FlywheelTask flywheelTask;
    public CapBallTask capBallTask;
    public IntakeTask intakeTask;
    public ButtonPusherTask buttonPusherTask;
    public double voltageLevel;

    @Override
    public void runOpMode() {

        initialize();
        double mc7 = hardwareMap.voltageSensor.get("frontDrive").getVoltage();
        double mc6 = hardwareMap.voltageSensor.get("backDrive").getVoltage();
        double mc3 = hardwareMap.voltageSensor.get("cap").getVoltage();
        double mc2 = hardwareMap.voltageSensor.get("flywheels").getVoltage();
        voltageLevel = (mc7 + mc6 + mc3 + mc2) / 4;
        driveTrainTask = new DriveTrainTask(this, frontLeft, frontRight, backLeft, backRight);
        flywheelTask = new FlywheelTask(this, flywheelLeft, flywheelRight);
        flywheelTask.voltage = voltageLevel;
        intakeTask = new IntakeTask(this);
        capBallTask = new CapBallTask(this);
        buttonPusherTask = new ButtonPusherTask(this, button, guide);
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
        capBottom = hardwareMap.dcMotor.get("capBottom");
        capTop = hardwareMap.dcMotor.get("capTop");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");

        button = hardwareMap.crservo.get("button");
        button.setPower(-0.44);
        forkLeft = hardwareMap.servo.get("forkLeft");
        forkRight = hardwareMap.servo.get("forkRight");
        guide = hardwareMap.servo.get("guide");
        //lift = hardwareMap.dcMotor.get("lift");
        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu = new VOIImu(adaImu);
        sweeper1 = hardwareMap.crservo.get("sweeper1");
        sweeper2 = hardwareMap.crservo.get("sweeper2");
        sweeper3 = hardwareMap.crservo.get("sweeper3");
        guide = hardwareMap.servo.get("guide");
        sweeper = new VOISweeper(sweeper1, sweeper2, sweeper3);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        capBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        capTop.setDirection((DcMotorSimple.Direction.REVERSE));
        forkLeft.setPosition(0.55);
        forkRight.setPosition(0.12);
        guide.setPosition(0.7);

        //gyro.calibrate();
        //gyro.resetZAxisIntegrator();
        //int base = gyro.getIntegratedZValue();
        //gate.setPosition(0.4);
        button.setPower(ButtonPusherTask.zeroPower);

    }
}
