package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by bunnycide on 10/13/16.
 */

@TeleOp(name = "Tele", group = "Drive")

public class Tele extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, flywheelRight, flywheelLeft, conveyor, sweeper;
    Servo gate, button;
    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {
        boolean increased = false ,decreased = false;
        boolean cIncreased = false, cDecreased = false;
        boolean buttonOut = false, xPushed = false;
        boolean gateOut = false, dpadUpPushed = false;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            if (timer.time()>500 && !gamepad2.y){
                button.setPosition(0);
                buttonOut = false;
            }
            telemetry.addData("Flywheel", flywheelRight.getPower());
            telemetry.addData("Conveyor", conveyor.getPower());

            //no PID
            double joy1Y = -gamepad1.left_stick_y;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
            double joy1X = gamepad1.left_stick_x;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
            double joy2X = gamepad1.right_stick_x;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
            if(Math.abs(joy1Y)==0 && Math.abs(joy1X)==0 && Math.abs(joy2X)==0){
                joy2X = 0.3*(gamepad2.right_trigger-gamepad2.left_trigger);
                if(joy2X==0) {
                    joy2X = 0.3 * (gamepad1.right_trigger - gamepad1.left_trigger);
                }
                joy1Y = -gamepad2.left_stick_y;
                joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*0.3: 0;
                joy1X = gamepad2.left_stick_x;
                joy1X = Math.abs(joy1X) > 0.15 ? joy1X*0.3: 0;
                if(joy1X==0 && joy1Y==0){
                    if(gamepad1.left_bumper){
                        joy1X = -0.3;
                    }
                    if(gamepad1.right_bumper){
                        joy1X = 0.3;
                    }
                }
            }
            frontLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X + joy1X)));
            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));
            if (gamepad1.x && !xPushed && !buttonOut){
                buttonOut = true;
                button.setPosition(1);
                timer.reset();
                xPushed = true;
            }
            if (!gamepad1.x){
                xPushed = false;
            }
            if(gamepad2.right_bumper){
                buttonOut = true;
                button.setPosition(1);
            }
            if(gamepad2.left_bumper){
                buttonOut = false;
                button.setPosition(0);
            }
            if (gamepad1.dpad_up && !dpadUpPushed){
                if (!gateOut){
                    gateOut = true;
                    gate.setPosition(0);
                }else{
                    gateOut = false;
                    gate.setPosition(0.4);
                }
                dpadUpPushed = true;
            }
            if (!gamepad1.dpad_up){
                dpadUpPushed = false;
            }
            /*
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
            }*/
            if (gamepad2.a){
                flywheelRight.setPower(0.7);
                flywheelLeft.setPower(flywheelRight.getPower());
            }
            else if (gamepad2.x){
                setPow(flywheelRight, 0);
                flywheelLeft.setPower(flywheelRight.getPower());
            }
            else if (gamepad2.y){
                buttonOut = true;
                button.setPosition(1);
                timer.reset();
            }
            else if (gamepad2.b){
                setPow(flywheelRight, 1.0);
                flywheelLeft.setPower(flywheelRight.getPower());
            }
            if(gamepad2.dpad_left || gamepad2.dpad_right){
                conveyor.setPower(0);
            }
            else if(gamepad2.dpad_up){
                conveyor.setPower(0.3);
            }
            else if(gamepad2.dpad_down){
                conveyor.setPower(-0.3);
            }
            if (!gamepad2.y){
                button.setPosition(0);
                buttonOut = false;
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
    public void setPow(DcMotor motor, double power){
        motor.setPower(power*0.7);
    }

}
