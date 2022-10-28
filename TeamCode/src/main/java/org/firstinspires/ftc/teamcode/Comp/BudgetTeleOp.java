/*
 * MECANUM/HOLOMETRIC MODE TELE OP MODE
 */

package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

@TeleOp(name="BudgetTeleOp", group="Comp")
public class BudgetTeleOp extends LinearOpMode {

    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);
    @Override
    public void runOpMode() {
        robot.init();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            boolean dYup = gamepad1.dpad_up;
            boolean dYdown = gamepad1.dpad_down;

            double gripPower = gamepad1.left_trigger; //servo

            int currentPosition = robot.arm.getCurrentPosition();
            //Servo to open/close hand
            if(dYup)
                robot.arm.setPower(0.5);
            else if(dYdown)
                robot.arm.setPower(-0.5);
            else {
                robot.arm.setPower(0);
                robot.arm.setTargetPosition(currentPosition);
            }




            //Used to ensure same ratio and contain values between [-1,1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


//ruight trigger =  claw //left trigger = slow down
            double throtte_control = .5;
            double slowDown = 1;
            if(gamepad1.left_trigger == 1) //if the button is pressed
                slowDown -= 0.25;


            robot.motor1.setPower(frontLeftPower*throtte_control*slowDown);
            robot.motor2.setPower(backLeftPower*throtte_control*slowDown);
            robot.motor3.setPower(frontRightPower*throtte_control*slowDown);
            robot.motor4.setPower(backRightPower*throtte_control*slowDown);
            robot.hand.setPosition(gripPower);

            telemetry.addData("frontLeft:", frontLeftPower);
            telemetry.addData("backLeft:", backLeftPower);
            telemetry.addData("frontRight:", frontRightPower);
            telemetry.addData("backRight:", backRightPower);
            telemetry.update();

        }
    }


    /*void motorTelemetry() {
        telemetry.addData("X", gamepad1.right_stick_x);
        telemetry.addData("Front", gamepad1.right_stick_x);
        telemetry.addData("Front", gamepad1.right_stick_x);
    }*/
}

