package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBarthRobot;
import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;
@Config
@TeleOp(name="EncoderValueTesting", group="Testing")
//@Disabled
public class EncoderValueTesting extends LinearOpMode {
    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motor1;
    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "Motor 1");
        waitForStart();
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(1000);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(.5);
        while(opModeIsActive() && motor1.isBusy())
            {
                telemetry.addData("Position", motor1.getCurrentPosition());
                telemetry.update();
            }

        robot.motor1.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}