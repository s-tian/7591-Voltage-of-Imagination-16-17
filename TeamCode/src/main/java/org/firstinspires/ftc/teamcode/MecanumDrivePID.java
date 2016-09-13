package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Mecanum Drive PID", group = "Tests")

public class MecanumDrivePID extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    ModernRoboticsI2cGyro gyro;



    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {

            double joy1Y = -gamepad1.left_stick_y;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
            double joy1X = gamepad1.left_stick_x;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
            double joy2X = gamepad1.right_stick_x;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
            int gyroValue = gyro.getIntegratedZValue();

            if((joy1X > 0 || joy1Y > 0) && joy2X == 0) {        //Lock on to the current gyro reading
                gyro.resetZAxisIntegrator();
                double KP = 0.03f;
                double KD = 0.00001f;
                double KI = 0.00000000001f;
                double I = 0;
                double D;
                double P;
                long lastTime = System.currentTimeMillis();
                int prevErr = 0;

                while((joy1X > 0 || joy1Y > 0) && joy2X == 0) {
                    //Update joystick to make sure that the action is still occurring

                    joy1Y = -gamepad1.left_stick_y;
                    joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
                    joy1X = gamepad1.left_stick_x;
                    joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
                    joy2X = gamepad1.right_stick_x;
                    joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
                    long currentTime = System.currentTimeMillis();
                    double delta = currentTime-lastTime;

                    int err = -gyro.getHeading();
                    P = err*KP;
                    I += err*delta*KI;
                    D = (err-prevErr)/delta*KD;
                    System.out.println("P: " + P + "I: " + I + "D: " + D);

                    double output = P + I + D;
                    frontLeft.setPower(range(joy1Y + joy2X + joy1X + output));
                    backLeft.setPower(range(joy1Y + joy2X - joy1X - output));
                    frontRight.setPower(range(joy1Y - joy2X - joy1X - output));
                    backRight.setPower(range(joy1Y - joy2X + joy1X + output));

                    prevErr = err;
                    lastTime = currentTime;
                }

            } else {
                frontLeft.setPower(joy2X);
                backLeft.setPower(joy2X);
                frontRight.setPower(-joy2X);
                backRight.setPower(-joy2X);
            }



        }
    }

    private double range(double x) {
        return Math.max(-1, Math.min(1, x));
    }
}
