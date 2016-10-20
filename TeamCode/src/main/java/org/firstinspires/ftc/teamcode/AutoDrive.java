package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

import static java.lang.Thread.sleep;

/**
 * Created by Stephen on 10/4/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoDrive", group = "Tests")
public class AutoDrive extends LinearOpMode {
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensorBottom;
    ColorSensor colorSensorTop;
    VOIColorSensor voiColorSensorBottom;
    VOIColorSensor voiColorSensorTop;
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        driveTrain.powerAll(0.2);
        while(!voiColorSensorBottom.isWhite()){
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            telemetry.addData("Color: ", red + " " + green + " " + blue);
            updateTelemetry(telemetry);
        }
        driveTrain.stopAll();
        driveTrain.rotateCounterClockwiseDegrees(45);
        driveTrain.strafeRight(0.2);
        sleep(500);
        driveTrain.stopAll();
    }
    public void testColor(){
        while(opModeIsActive()){
            int red = colorSensorBottom.red();
            int green = colorSensorBottom.green();
            int blue = colorSensorBottom.blue();
            telemetry.addData("Bottom: ", red + " " + green + " " + blue);
            telemetry.addData("Top: ", colorSensorTop.red() + " " + colorSensorTop.green() + " " + colorSensorTop.blue());
            telemetry.addData("White: ", voiColorSensorBottom.isWhite() || voiColorSensorBottom.isWhite());
            System.out.println("Top: " + colorSensorTop.red() + " " + colorSensorTop.green() + " " + colorSensorTop.blue());
            System.out.println("Bottom: " + red + " " + green + " " + blue);
            System.out.println("White: " + (voiColorSensorTop.isWhite() || (voiColorSensorBottom.isWhite())));
            updateTelemetry(telemetry);
        }
    }
    public void initialize(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        colorSensorBottom = hardwareMap.colorSensor.get("colorBottom");
        colorSensorTop = hardwareMap.colorSensor.get("colorTop");
        final int COLORSENSORBOTI2CADDR = 0x44;
        final int topSensorID = 0x3c;
        final int bottomSensorID = 0x44;
        colorSensorBottom.setI2cAddress(I2cAddr.create8bit(topSensorID));//maybe create8bit
        colorSensorTop.setI2cAddress(I2cAddr.create8bit(bottomSensorID));
        //colorSensorBottom.setI2cAddress(0x42);
        voiColorSensorBottom = new VOIColorSensor(colorSensorBottom);
        voiColorSensorTop = new VOIColorSensor(colorSensorTop);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        gyro.resetZAxisIntegrator(); //address is 0x20
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new MecanumDriveTrain(backLeft,backRight,frontLeft,frontRight,gyro,this);
        driveTrain.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
