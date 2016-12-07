package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Stephen on 10/6/2016.
 */

//@TeleOp(name = "Conveyor Test", group = "Test")

public class ConveyorTest extends LinearOpMode {

    DcMotor motorA;

    @Override
    public void runOpMode() throws InterruptedException{
        motorA = hardwareMap.dcMotor.get("motorA");
        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            motorA.setPower(gamepad1.left_stick_y);
        }
    }
}
