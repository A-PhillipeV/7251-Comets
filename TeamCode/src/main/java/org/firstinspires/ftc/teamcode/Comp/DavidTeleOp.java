package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBarthRobot;

@TeleOp(name="DavidTeleOp", group="Comp")
@Disabled
public class DavidTeleOp extends LinearOpMode {

    // Declare OpMode members.
    HardwareBarthRobot robot = new HardwareBarthRobot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double handPower;
        double platformPower = 0.0;
        double elbowPower = 0.0;
        double speed = 0;
        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;

            robot.motorLeft.setPower(y + x);
            robot.motorRight.setPower(y - x);


            /* TELEMETRY */
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("gampad 1 y", y);
            telemetry.addData("gampad 1 x", x);
            telemetry.update();
        }
    }

    class Speed{
        private double vel;

        public Speed() {
            vel = 0.5;
        }

        public double getVel() {
            return vel;
        }

        public void downVel() {
            if (vel > 0) {
                vel -= 0.01;
            }
        }

        public void upVel() {
            if (vel < 1) {
                vel += 0.01;
            }
        }
    }
}
