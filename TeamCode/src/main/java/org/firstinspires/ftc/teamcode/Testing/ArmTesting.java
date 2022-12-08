package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

@Config
@TeleOp(name="ArmTesting", group="Testing")
//@Disabled
public class ArmTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor arm;
    final int lowHeight = 5800;
    final int middleHeight = 8400;


    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        waitForStart();

        while(opModeIsActive())
            {
                double power = 1;
                //If Y is pressed middle height
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setTargetPosition(0);


                if(gamepad1.y) {
                    arm.setTargetPosition(-1000);
                    arm.setPower(power);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }

                else if(gamepad1.x){
                    arm.setTargetPosition(1000);
                    arm.setPower(power);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
                else {
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                telemetry.addData("Position", arm.getCurrentPosition());
                telemetry.update();
                }

            }

    }