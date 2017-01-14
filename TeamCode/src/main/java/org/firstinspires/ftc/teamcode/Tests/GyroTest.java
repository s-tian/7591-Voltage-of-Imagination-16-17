package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Gyro Test", group = "Test")
@Disabled

public class GyroTest extends LinearOpMode {

    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Gyro reading: ", gyro.getIntegratedZValue());
            telemetry.update();
        }
    }

}
