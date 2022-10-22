package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DavidTest", group = "Testing")

public class TestingOpMode<opModeIsActive> extends LinearOpMode {
    DcMotor lm;
    DcMotor rm;
    DcMotor motor3;

    @Override
    public void runOpMode() {
        lm = hardwareMap.get(DcMotor.class, "Left_Motor");
        rm = hardwareMap.get(DcMotor.class, "Right_Motor");
        motor3 = hardwareMap.get(DcMotor.class, "Elbow");
        lm.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {


        }
    }
}