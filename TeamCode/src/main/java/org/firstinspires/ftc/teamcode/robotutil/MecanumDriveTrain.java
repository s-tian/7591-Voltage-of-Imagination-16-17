package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by Stephen on 9/17/2016.
 */
public class MecanumDriveTrain {
    public static double INCHES_PER_1000_TICKS_FORWARD = 19.03;
    public static double INCHES_PER_1000_TICKS_STRAFE = 13.34;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    ModernRoboticsI2cGyro gyro;
    LinearOpMode opMode;
    public DcMotor[] motorArray = new DcMotor[4];

    public MecanumDriveTrain(DcMotor backLeft, DcMotor backRight, DcMotor frontLeft, DcMotor frontRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        reverseMotors();
        motorArray[0] = backLeft;
        motorArray[1] = backRight;
        motorArray[2] = frontLeft;
        motorArray[3] = frontRight;
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
        setMotorPower(backLeft, power);
        setMotorPower(backRight, power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, power);
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
    public void strafeLeft(double power) {
        setMotorPower(backRight, (float) -power);
        setMotorPower(backLeft, (float) power);
        setMotorPower(frontLeft, (float) -power);
        setMotorPower(frontRight, (float) power);
    }
    public void strafeRight(double power){
        setMotorPower(backRight, (float)power);
        setMotorPower(backLeft, (float)-power);
        setMotorPower(frontLeft, (float)power);
        setMotorPower(frontRight, (float)-power);
    }

    public void rotateClockwiseDegrees(double degrees) throws InterruptedException{
        startClockWiseRotation(0.1);
        while (Math.abs(gyro.getIntegratedZValue() + degrees) != 0 && opMode.opModeIsActive()){
            double gyroValue = gyro.getIntegratedZValue();
            double velocity;
            if (gyroValue > -degrees)
                velocity = Math.max((degrees + gyroValue)/degrees, 0.1);
            else{
                velocity = Math.min((degrees + gyroValue)/degrees, -0.1);
            }
            startClockWiseRotation(velocity);
        }
        stopAll();
    }
    public void rotateCounterClockwiseDegrees(double degrees) throws InterruptedException{
        startClockWiseRotation(0.35);
        while (Math.abs(gyro.getIntegratedZValue() + degrees) != 0 && opMode.opModeIsActive()){
            System.out.println("getIntegratedZValue: " + gyro.getIntegratedZValue());
            double gyroValue = gyro.getIntegratedZValue();
            double velocity;
            if (degrees > gyroValue)
                velocity = Math.max((degrees + gyroValue)/degrees, 0.2);
            else{
                velocity = Math.min((degrees + gyroValue)/degrees, -0.2);
            }
            startCounterClockWiseRotation(velocity);
        }
        stopAll();
    }
    public void rotateWithoutAdjustment(double power, double degrees) throws InterruptedException{
        int initialGyro = gyro.getIntegratedZValue();
        if (degrees > 0){
            startCounterClockWiseRotation(power);
            while(gyro.getIntegratedZValue() > initialGyro - degrees && opMode.opModeIsActive()){
                System.out.println(gyro.getIntegratedZValue() - initialGyro);
            }
        }
        else{
            startClockWiseRotation(power);
            while(gyro.getIntegratedZValue() < initialGyro + degrees && opMode.opModeIsActive()) {
                System.out.println(gyro.getIntegratedZValue() - initialGyro);
            }
        }
    }
    public void moveForwardNInch(float power, float inches)
            throws InterruptedException {
        moveForwardTicksWithEncoders(power, (int) (inches*(1000/INCHES_PER_1000_TICKS_FORWARD)));
        stopAll();
    }
    public void moveBackwardNInch(float power, float inches)
            throws InterruptedException {
        moveBackwardTicksWithEncoders(power, (int) (inches*(1000/INCHES_PER_1000_TICKS_FORWARD)));
        stopAll();
    }
    public void moveLeftNInch(double power, double inches) throws InterruptedException{
        int initialDegrees = gyro.getIntegratedZValue();
        moveLeftTicksWithEncoders(power, (int) (inches*1000/INCHES_PER_1000_TICKS_STRAFE));
        int finalDegrees = gyro.getIntegratedZValue();
        rotateWithoutAdjustment(0.1,finalDegrees-initialDegrees);
        System.out.println(finalDegrees - initialDegrees);
    }
    public void moveRightNInch(double power, double inches) throws InterruptedException{
        int initialDegrees = gyro.getIntegratedZValue();
        moveRightTicksWithEncoders(power, (int) (inches*1000/INCHES_PER_1000_TICKS_STRAFE));
        int finalDegrees = gyro.getIntegratedZValue();
        rotateWithoutAdjustment(0.1, finalDegrees - initialDegrees);
    }
    public void moveLeftTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int initialBackRight = backRight.getCurrentPosition();
        strafeLeft(power);
        while(opMode.opModeIsActive()) {
            if (backRight.getCurrentPosition() <= initialBackRight - ticks) {
                stopAll();
                return;
            }
        }
    }
    public void moveRightTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int initialBackRight = backRight.getCurrentPosition();
        strafeLeft(power);
        while(opMode.opModeIsActive()) {
            if (backRight.getCurrentPosition() >= initialBackRight + ticks) {
                stopAll();
                return;
            }
        }
    }
    public void moveForwardTicksWithEncoders(float power, int ticks) throws InterruptedException{
        moveTicks(power, ticks,true);
    }
    public void moveBackwardTicksWithEncoders(float power, int ticks) throws InterruptedException{
        moveTicks(power, ticks, false);
    }
    public void getTicks(){
        System.out.println(backRight.getCurrentPosition());
    }
    public void moveTicks(float power, int ticks, boolean forward) throws InterruptedException{
        int initialBackRight = backRight.getCurrentPosition();
//        int initialBackLeft = backLeft.getCurrentPosition();
//        int initialFrontLeft = frontLeft.getCurrentPosition();
//        int initialFrontRight = frontRight.getCurrentPosition();
        powerAllMotors(forward?power:-power);

//        boolean runBackRight = true;
//        boolean runBackLeft = true;
//        boolean runFrontRight = true;
//        boolean runFrontLeft = true;

            while (opMode.opModeIsActive()) {
                if (forward) {
                    if (backRight.getCurrentPosition() >= ticks + initialBackRight) {
                        //runBackRight = false;
                        stopAll();
                        return;
                        //backRight.setPower(0);
                    }
//                    if (backLeft.getCurrentPosition() >= ticks + initialBackLeft) {
//                        runBackLeft = false;
//                        backLeft.setPower(0);
//                    }
//                    if (frontLeft.getCurrentPosition() >= ticks + initialFrontLeft) {
//                        runFrontLeft = false;
//                        frontLeft.setPower(0);
//                    }
//                    if (frontRight.getCurrentPosition() >= ticks + initialFrontRight) {
//                        runFrontRight = false;
//                        frontRight.setPower(0);
//                    }
                }else{
                    if (backRight.getCurrentPosition() <= -ticks + initialBackRight) {
                        //runBackRight = false;
                        //backRight.setPower(0);
                        stopAll();
                        return;
                    }
//                    if (backLeft.getCurrentPosition() <= -ticks + initialBackLeft) {
//                        runBackLeft = false;
//                        backLeft.setPower(0);
//                    }
//                    if (frontLeft.getCurrentPosition() <= -ticks + initialFrontLeft) {
//                        runFrontLeft = false;
//                        frontLeft.setPower(0);
//                    }
//                    if (frontRight.getCurrentPosition() <= -ticks + initialFrontRight) {
//                        runFrontRight = false;
//                        frontRight.setPower(0);
//                    }
                }
            }

    }
    public void powerAllForTime(float power, float seconds)
            throws InterruptedException
    {
        powerAllMotors(power);
        Thread.sleep((long) (1000*seconds));
        stopAll();
    }

}
