package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;
@Config
@TeleOp(name="EncoderValueTesting", group="Testing")
//@Disabled
public class EncoderValueTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);
    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();


        /* Write program to go to height 5400 (encoder tickers) and let it stay */


    }
}