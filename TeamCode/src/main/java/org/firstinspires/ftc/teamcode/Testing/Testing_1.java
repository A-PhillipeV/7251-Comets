package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorRunning", group = "Testing")

public class Testing_1<opModeIsActive> extends LinearOpMode {
    DcMotor lm;
    DcMotor rm;
    DcMotor motor3;

    @Override
    public void runOpMode() {
        lm = hardwareMap.get(DcMotor.class, "Motor 4");

        waitForStart();


        while (opModeIsActive()) {
            lm.setPower(.5);
        }
    }
}
