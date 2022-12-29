package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

@Config
@TeleOp(name="Motor Encoder Test", group="Testing")
//@Disabled
public class MotorEncoderTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "Motor 1");
        motor2 = hardwareMap.get(DcMotor.class, "Motor 2");
        motor3 = hardwareMap.get(DcMotor.class, "Motor 3");
        motor4 = hardwareMap.get(DcMotor.class, "Motor 4");

        waitForStart();

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {
            telemetry.addData("Motor 1", motor1.getCurrentPosition());
            telemetry.addData("Motor 2", motor2.getCurrentPosition());
            telemetry.addData("Motor 3", motor3.getCurrentPosition());
            telemetry.addData("Motor 4", motor4.getCurrentPosition());

            telemetry.update();
        }
    }

}
