package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

/**
 * Created by Howard on 10/23/16.
 */
@TeleOp(name = "Drive Test", group = "Tests")

public class DriveTest extends LinearOpMode {
    MecanumDriveTrain driveTrain;
    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        driveTrain.moveRightNInch(0.3, 30, 20);

    }
    public void initialize(){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        driveTrain = new MecanumDriveTrain(backLeft, backRight, frontLeft, frontRight, this);
    }
    public void testTicks(){
        driveTrain.getTicks();
        driveTrain.strafeRight(1);
        sleep(1500);
        driveTrain.stopAll();
        driveTrain.getTicks();
    }
}
