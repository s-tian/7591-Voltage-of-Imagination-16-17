package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by Stephen on 9/17/2016.
 */
public class MecanumDriveTrain {
    public static final double TICKS_PER_INCH_FORWARD= 57.89;
    public static final double TICKS_PER_INCH_STRAFE = 61.05;
    public static ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static final double POWER_RATIO = 0.78;
    public DcMotor backLeft, backRight, frontLeft, frontRight;
    ModernRoboticsI2cGyro gyro;
    LinearOpMode opMode;
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
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gyro = gyro;
        this.opMode = opMode;
        reverseMotors();
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
    public void powerAll(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void setMotorPower(DcMotor motor,double power){
        motor.setPower(POWER_RATIO*power);
    }
    public void powerAllMotors(double power){
        setMotorPower(backLeft, power);
        setMotorPower(backRight, power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, power);
    }
    public void stopAll() {
        powerAll(0);
    }
    public void powerRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(frontRight, power);
    }
    public void powerLeft(double power){
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, power);
    }
    public void startRotation(double power){
        powerLeft(power);
        powerRight(-power);
    }
    public void strafeLeft(double power) {
        setMotorPower(backRight,  -power);
        setMotorPower(backLeft, power);
        setMotorPower(frontLeft, -power);
        setMotorPower(frontRight, power);
    }
    public void strafeRight(double power){
        setMotorPower(backRight, power);
        setMotorPower(backLeft, -power);
        setMotorPower(frontLeft, power);
        setMotorPower(frontRight, -power);
    }
    public void rotateDegrees(int degrees) throws InterruptedException{
        // Clockwise: degrees > 0
        // CounterClockwise: degrees < 0;
        System.out.println(gyro.getIntegratedZValue());
        int targetGyro = gyro.getIntegratedZValue() + degrees;
        double velocity;
        while (Math.abs(gyro.getIntegratedZValue() - targetGyro) > 2 && opMode.opModeIsActive()){
            double gyroValue = gyro.getIntegratedZValue();
            if (gyroValue < targetGyro)
                velocity = Math.max((targetGyro - gyroValue)*0.55/degrees, 0.2);
            else
                velocity = Math.min((targetGyro-gyroValue)*0.55/degrees, -0.2);

            startRotation(velocity);
        }
        stopAll();
        System.out.println(gyro.getIntegratedZValue());
    }
    public void moveForwardNInch(double power, double inches) throws InterruptedException {
        moveForwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD));
    }
    public void moveBackwardNInch(double power, double inches) throws InterruptedException {
        moveBackwardTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_FORWARD));
    }
    public void moveLeftNInch(double power, double inches, int timeout) throws InterruptedException{
        moveLeftTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout);
}
    public void moveRightNInch(double power, double inches, int timeout) throws InterruptedException{
        moveRightTicksWithEncoders(power, (int) (inches*TICKS_PER_INCH_STRAFE), timeout);
    }
    public void moveLeftTicksWithEncoders(double power, int ticks, int timeout) throws InterruptedException{
        int timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() - ticks;
        strafeLeft(power);
        timer.reset();
        while(backRight.getCurrentPosition() > targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {}
        timer.reset();
        stopAll();
    }
    public void moveRightTicksWithEncoders(double power, int ticks, int timeout) throws InterruptedException{
        int timeOutMS = timeout*1000;
        int targetPosition = backRight.getCurrentPosition() + ticks;
        strafeRight(power);
        timer.reset();
        while(backRight.getCurrentPosition() < targetPosition && opMode.opModeIsActive() && timer.time() < timeOutMS) {}
        timer.reset();
        stopAll();
    }
    public void moveForwardTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int target = backRight.getCurrentPosition() + ticks;
        powerAllMotors(power);
        while (opMode.opModeIsActive() && backRight.getCurrentPosition() < target) {}
        stopAll();
    }
    public void moveBackwardTicksWithEncoders(double power, int ticks) throws InterruptedException{
        int target = backRight.getCurrentPosition() - ticks;
        powerAllMotors(-power);
        while (opMode.opModeIsActive() && backRight.getCurrentPosition() > target) {}
        stopAll();
    }
    public void getTicks(){
        System.out.println(backRight.getCurrentPosition());
    }
}
