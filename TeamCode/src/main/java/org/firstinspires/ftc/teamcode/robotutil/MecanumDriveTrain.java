package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by Stephen on 9/17/2016.
 */
public class MecanumDriveTrain {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    ModernRoboticsI2cGyro gyro;
    LinearOpMode opMode;
    public DcMotor[] motorArray = {backLeft, backRight, frontLeft, frontRight};

    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        reverseMotors();
    }
    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight, ModernRoboticsI2cGyro gyro, LinearOpMode opMode) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.gyro = gyro;
        this.opMode = opMode;
        reverseMotors();
    }

    public enum DriveTrainMotor {
        BACK_LEFT, BACK_RIGHT, FRONT_LEFT ,FRONT_RIGHT;
    }

    private void reverseMotors() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setEncoderMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
    }


    public void setMotorPower(DriveTrainMotor motor, double power) {
        switch(motor) {
            case BACK_LEFT:
                backLeft.setPower(power);
                break;
            case BACK_RIGHT:
                backRight.setPower(power);
                break;
            case FRONT_LEFT:
                frontLeft.setPower(power);
                break;
            case FRONT_RIGHT:
                frontRight.setPower(power);
        }
    }

    public void powerAll(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void stopAll() {
        powerAll(0);
    }

    public void strafeLeftCorrected(double power) {

    }
    public void setMotorPower(DcMotor motor,float power){
        motor.setPower(power);
    }
    public void setMotorPower(MecanumDriveTrain.DriveTrainMotor motor, float power){
        switch(motor){
            case BACK_LEFT:
                setMotorPower(backLeft, power);
                break;
            case BACK_RIGHT:
                setMotorPower(backRight, power);
                break;
            case FRONT_LEFT:
                setMotorPower(frontLeft, power);
                break;
            case FRONT_RIGHT:
                setMotorPower(frontRight, power);
        }
    }
    public void powerAllMotors(float power){
        for (DcMotor motor : motorArray){
            setMotorPower(motor, power);
        }
    }
    public void powerRight(float power){
        setMotorPower(backRight, power);
        setMotorPower(frontRight, power);
    }
    public void powerLeft(float power){
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, power);
    }
    public void startClockWiseRotation(double power){
        powerLeft((float) power);
        powerRight((float) -power);
    }
    public void startCounterClockWiseRotation(double power){
        powerLeft((float) -power);
        powerRight((float) power);
    }
    public void strafeLeft(double power){
        setMotorPower(backRight, (float)-power);
        setMotorPower(backLeft, (float)power);
        setMotorPower(frontLeft, (float)-power);
        setMotorPower(frontRight, (float)power);
    }
    public void strafeRight(double power){
        setMotorPower(backRight, (float)power);
        setMotorPower(backLeft, (float)-power);
        setMotorPower(frontLeft, (float)power);
        setMotorPower(frontRight, (float)-power);
    }
    public void rotateClockwiseDegrees(double degrees) throws InterruptedException{
        startClockWiseRotation(0.5);
        while (gyro.getIntegratedZValue() > -degrees && opMode.opModeIsActive()) {
            System.out.println("getHeading: " + gyro.getHeading());
            //opMode.telemetry.addData("integratedZValue: ", gyro.getIntegratedZValue());
            //opMode.updateTelemetry(opMode.telemetry);
        }
        stopAll();
    }




}
