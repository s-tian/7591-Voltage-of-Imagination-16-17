package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Stephen on 11/29/2016.
 */
@TeleOp(name = "VEX Test", group = "Test")

public class VEXMotorTest extends LinearOpMode {
    CRServo vexMotor;
    CRServo vexM2;

    @Override

    public void runOpMode() {
        vexMotor = hardwareMap.crservo.get("sweeper1");
        vexM2 = hardwareMap.crservo.get("sweeper2");

        waitForStart();
        while(opModeIsActive()) {
            vexMotor.setPower(1);
            vexM2.setPower(1);
        }
    }
}
