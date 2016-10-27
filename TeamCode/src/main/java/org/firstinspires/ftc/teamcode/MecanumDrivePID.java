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

//@TeleOp(name = "Mecanum Drive PID", group = "Tests")

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

        gyro.resetZAxisIntegrator();
        int base = gyro.getIntegratedZValue();

        waitForStart();
        while(opModeIsActive()) {

            double joy1Y = -gamepad1.left_stick_y;
            double joy1X = gamepad1.left_stick_x;
            double joy2X = gamepad1.right_stick_x;

            if((Math.abs(joy1X) > 0.05 || Math.abs(joy1Y) > 0.05) && joy2X == 0 && opModeIsActive()) {

                double KP = 0.01;
                double KD = 0.001;
                double KI = 0;
                double I = 0;
                double D;
                double P;
                long lastTime = System.currentTimeMillis();
                int prevErr = 0;
                base = -gyro.getIntegratedZValue();
                while(opModeIsActive() && (Math.abs(joy1X) > 0 || Math.abs(joy1Y) > 0) && joy2X == 0) {
                    //Update joystick to make sure that the action is still occurring
                    joy1Y = -gamepad1.left_stick_y*3/4;
                    joy1X = gamepad1.left_stick_x*3/4;
                    joy2X = gamepad1.right_stick_x*3/4;
                    //long currentTime = System.currentTimeMillis();
                    //double delta = currentTime-lastTime;

                    int err = -gyro.getIntegratedZValue() - base;
                    System.out.println(err);
                    P = err*KP;
//                    I += err*delta*KI;
//                    if (delta == 0) {
//                        delta = 1;
//                    }
//                    D = (err-prevErr)/delta*KD;
//                    double output = P + I + D;
                    frontLeft.setPower(crange(joy1Y + joy1X - P));
                    backLeft.setPower(crange(joy1Y - joy1X - P));
                    frontRight.setPower(crange(joy1Y - joy1X + P));
                    backRight.setPower(crange(joy1Y + joy1X + P));
                    System.out.println("FL: " + (joy1Y + joy1X - P) + " BL: " + (joy1Y - joy1X - P) + " FR: " + (joy1Y - joy1X + P) + " BR: " + (joy1Y + joy1X + P));
//                    prevErr = err;
//                    lastTime = currentTime;
                }

            } else if(Math.abs(joy2X) > 0) {
                frontLeft.setPower(crange(joy1Y + joy2X + joy1X));
                backLeft.setPower(crange(joy1Y + joy2X - joy1X));
                frontRight.setPower(crange(joy1Y - joy2X - joy1X));
                backRight.setPower(crange(joy1Y - joy2X + joy1X));
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
        }
    }

    private double crange(double x) {
        return Math.min(1, Math.max(-1, x));
    }

}
