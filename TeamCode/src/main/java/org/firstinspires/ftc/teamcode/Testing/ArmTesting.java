package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

@Config
@TeleOp(name="ArmTesting", group="Testing")
//@Disabled
public class ArmTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor arm;
    CRServo hand;

    final int groundHeight = 3200;
    final int lowHeight = 5800;
    final int middleHeight = 8400;


    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        hand = hardwareMap.get(CRServo.class, "Hand");
        waitForStart();

        double power = 1;
        //If Y is pressed middle height
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {
            if (gamepad1.y) {
                arm.setTargetPosition(lowHeight);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad1.x) {
                arm.setTargetPosition(groundHeight);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad1.a){
                arm.setTargetPosition(0);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.b) {
                arm.setTargetPosition(groundHeight);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.left_stick_y * -1 > 0) {
                arm.setTargetPosition(arm.getCurrentPosition() + 300);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.left_stick_y * -1 < 0) {
                arm.setTargetPosition(arm.getCurrentPosition() - 300);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.right_trigger > 0) {
                hand.setPower(1);
            }
            else if(gamepad1.left_trigger > 0) {
                hand.setPower(-1);
            }
            else {
                hand.setPower(0);
            }


            telemetry.addData("Position", arm.getCurrentPosition());
            telemetry.update();
        }


    }

}
