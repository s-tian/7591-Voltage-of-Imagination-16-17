package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.Tests.ShooterTest.KMode.KP;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Shooter Test", group = "Test")

public class ShooterTest extends LinearOpMode {

    FlywheelTask flywheelTask;
    DcMotor flywheelRight, flywheelLeft;
    IntakeTask intakeTask;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public enum KMode {
        KP, KI, KD
    }

    @Override
    public void runOpMode() throws InterruptedException {
        flywheelTask = new FlywheelTask(this);
        intakeTask = new IntakeTask(this);
        new CapBallTask(this);
        TaskThread.calculateVoltage(this);
        flywheelTask.teleOp = intakeTask.teleOp = true;
        initialize();
        //setPID();
        setPowers();
        waitForStart();
        flywheelTask.start();
        intakeTask.start();
        //runFullPower();
        timer.reset();
        int prevRight = flywheelRight.getCurrentPosition();
        int prevLeft = flywheelLeft.getCurrentPosition();
        while(opModeIsActive()) {
            if (timer.time() > 200) {
                System.out.println("Right: " + (flywheelRight.getCurrentPosition() - prevRight)/timer.time()*1000);
                System.out.println("Left: " + (flywheelLeft.getCurrentPosition() - prevLeft)/timer.time()*1000);
                prevLeft = flywheelLeft.getCurrentPosition();
                prevRight = flywheelRight.getCurrentPosition();
                telemetry.update();
                timer.reset();
            }
        }
    }

    public void setPowers() {
        boolean confirmed = false;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;

        while (!confirmed) {
            if (gamepad2.a && !aPressed) {
                aPressed = true;
                FlywheelTask.lowPow += 0.01;
            }
            if (gamepad2.b && !bPressed) {
                bPressed = true;
                FlywheelTask.lowPow -= 0.01;
            }
            if (gamepad2.x && !xPressed) {
                xPressed = true;
                FlywheelTask.highPow += 0.01;
            }
            if (gamepad2.y && !yPressed) {
                yPressed = true;
                FlywheelTask.highPow -= 0.01;
            }

            if (!gamepad2.a){
                aPressed = false;
            }
            if (!gamepad2.b) {
                bPressed = false;
            }
            if (!gamepad2.x){
                xPressed = false;
            }
            if (!gamepad2.y){
                yPressed = false;
            }
            telemetry.addData("A (low)", FlywheelTask.lowPow);
            telemetry.addData("B (high)", FlywheelTask.highPow);
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();


        }
    }

    public void setPID() {
        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(8);
        KMode mode = KMode.KP;
        boolean confirmed = false;
        double changeValue = 0.001;
        boolean rBumper = false;
        boolean lBumper = false;
        boolean upPressed = false;
        boolean downPressed = false;
        while (!confirmed) {
            if (gamepad2.right_bumper && !rBumper) {
                rBumper = true;
                changeValue *= 10;
            }
            if (gamepad2.left_bumper && !lBumper) {
                lBumper = true;
                changeValue /= 10;
            }
            if (!gamepad2.right_bumper) {
                rBumper = false;
            }
            if (!gamepad2.left_bumper) {
                lBumper = false;
            }
            if (gamepad2.a) {
                mode = KMode.KP;
            } else if (gamepad2.b) {
                mode = KMode.KI;
            } else if (gamepad2.x) {
                mode = KMode.KD;
            }

            if (gamepad2.dpad_up && !upPressed) {
                upPressed = true;
                switch (mode) {
                    case KP:
                        FlywheelTask.KP += changeValue;
                        break;
                    case KI:
                        FlywheelTask.KI += changeValue;
                        break;
                    case KD:
                        FlywheelTask.KD += changeValue;
                        break;
                }
            }

            if (gamepad2.dpad_down && !downPressed) {
                downPressed = true;
                switch (mode) {
                    case KP:
                        FlywheelTask.KP -= changeValue;
                        break;
                    case KI:
                        FlywheelTask.KI -= changeValue;
                        break;
                    case KD:
                        FlywheelTask.KD -= changeValue;
                        break;
                }
            }
            if (!gamepad2.dpad_up) {
                upPressed = false;
            }
            if (!gamepad2.dpad_down) {
                downPressed = false;
            }
            String kMode = "";
            switch (mode) {
                case KP:
                    kMode = "KP";
                    break;
                case KI:
                    kMode = "KI";
                    break;
                case KD:
                    kMode = "KD";
                    break;
            }
            telemetry.addData("Change value", changeValue);
            telemetry.addData("Current variable", kMode);
            telemetry.addData("KP", df.format(FlywheelTask.KP));
            telemetry.addData("KI", df.format(FlywheelTask.KI));
            telemetry.addData("KD", df.format(FlywheelTask.KD));
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();

        }
    }

    public void runFullPower() {
        initialize();
        flywheelRight.setPower(0.5);
        flywheelLeft.setPower(0.5);
    }

    public void initialize() {
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
