package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class RoadRunnerTesting extends LinearOpMode {
    DcMotor arm;
    CRServo hand;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        hand = hardwareMap.get(CRServo.class, "Hand");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Path Starts Here

        /*
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(47,0, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    linearDrive(0.4, 2000);
                })
                .forward(3)
                .addDisplacementMarker(() -> {
                    openHand();
                })

                .build();

         */


        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(47,0, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    linearDrive(0.4, 2000);
                })

                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(3)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        //actual movement starts here
        closeHand();
        linearDrive(0.5, 300);
        drive.followTrajectory(trajectory1);
        drive.followTrajectory(trajectory2);//moves to junction
        openHand();
        //Move arm up and score





        //

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }


    public void linearDrive(double speed,
                            double target) {
        int newArmTarget;

        double armPower;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newArmTarget = arm.getCurrentPosition() + (int)(target);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            arm.setTargetPosition(-newArmTarget);

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.  (these values are for strafing right)
            //runtime.reset();


            armPower = speed;

            arm.setPower(armPower);

            while (opModeIsActive() &&
                    (arm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to (LINEAR DRIVE)",  "Arm: %7d ",
                        newArmTarget);
                telemetry.addData("Currently at (LINEAR DRIVE)",  "Arm: %7d",
                        arm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            //arm.setPower(0);

        }


    }
    //linear drive ends here - david elian valaarezo

    public void openHand() {
        hand.setPower(1);
        sleep(1750);
        hand.setPower(0);
    }

    public void closeHand() {
        hand.setPower(-1);
        sleep(1750);
        hand.setPower(0);
    }

}
