package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

/**
 * Created by bunnycide on 10/13/16.
 */

@TeleOp(name = "TeleOp", group = "Drive")

public class Integration extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, conveyor, sweeper;
    Servo gate, button;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean increased = false ,decreased = false;
        boolean cIncreased = false, cDecreased = false;

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

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Flywheel", flywheelRight.getPower());
            telemetry.addData("Conveyor", conveyor.getPower());

            //no PID
            double joy1Y = -gamepad1.left_stick_y;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
            double joy1X = gamepad1.left_stick_x;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
            double joy2X = gamepad1.right_stick_x;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
            frontLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X + joy1X)));
            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));

            if (gamepad1.right_trigger > 0 && flywheelRight.getPower() <= .9 && !increased) {
                flywheelRight.setPower(flywheelRight.getPower() + .1);
                flywheelLeft.setPower(flywheelRight.getPower());
                increased = true;
            }
            if (gamepad1.left_trigger > 0 && flywheelRight.getPower() >= .1 && !decreased) {
                flywheelRight.setPower(flywheelRight.getPower() - .1);
                flywheelLeft.setPower(flywheelRight.getPower());
                decreased = true;
            }
            if (gamepad1.right_bumper && conveyor.getPower() <= .9 && !cIncreased){
                conveyor.setPower(conveyor.getPower() + .1);
                cIncreased = true;
            }
            if (gamepad1.left_bumper && conveyor.getPower() >= .1 && !cDecreased){
                conveyor.setPower(conveyor.getPower()- .1);
                cDecreased = true;
            }
            if (gamepad1.right_trigger == 0){
                increased = false;
            }
            if (gamepad1.left_trigger == 0) {
                decreased = false;
            }
            if (!gamepad1.right_bumper){
                cIncreased = false;
            }
            if (!gamepad1.left_bumper){
                cDecreased = false;
            }
            if(gamepad1.y){
                sweeper.setPower(1);
            }
            if(gamepad1.a){
                sweeper.setPower(-1);
            }
            if(gamepad1.b){
                sweeper.setPower(0);
            }

            telemetry.update();
        }
    }
}
