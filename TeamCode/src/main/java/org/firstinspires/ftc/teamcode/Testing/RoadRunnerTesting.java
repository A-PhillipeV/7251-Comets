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
   DcMotor m1;
   DcMotor m2;
   DcMotor m3;
   DcMotor m4;

   @Override
   public void runOpMode() throws InterruptedException {
       arm = hardwareMap.get(DcMotor.class, "Arm");
       hand = hardwareMap.get(CRServo.class, "Hand");

       m1 = hardwareMap.get(DcMotor.class, "Motor 1");
       m2 = hardwareMap.get(DcMotor.class, "Motor 2");
       m3 = hardwareMap.get(DcMotor.class, "Motor 3");
       m4 = hardwareMap.get(DcMotor.class, "Motor 4");

       Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
       Pose2d startPose = new Pose2d(-34, 70, Math.toRadians(-90));
       drive.setPoseEstimate(startPose);

       //Getting first cone to medium junction
       TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                       .strafeLeft(32) //change to strafeLeft(34) don't think it matters tho
                       .lineToLinearHeading(new Pose2d(2,28,Math.toRadians(180)))
                       .forward(4) //comment out if raising arm once at medium junction
                       .addTemporalMarker(()->openHand())
                       .build();

       //place the first cone
       TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                       //.forward(4) //uncomment if raising arm after arriving at medium junction
                       .back(4)
                       .strafeLeft(14)
                       .addTemporalMarker(()->armMove(0.75, -2000))
                       .forward(49)
                       .waitSeconds(1)
                       .addTemporalMarker(()->closeHand())

                       //move from cone stack to medium junction
                       .lineToLinearHeading(new Pose2d(-3, 13, Math.toRadians(90)))
                       .addTemporalMarker(()->armMove(0.75, 2000))

                       //move forward and drop cone
                       .forward(1)
                       .addTemporalMarker(()->openHand())
                       .build();
       // spam cones after the second cone is placed
       TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                       .back(8)
                       .addTemporalMarker(()->armMove(0.75, -2100))
                       .lineToLinearHeading(new Pose2d(-44, 10.5, Math.toRadians(180)))
                       .build();

       waitForStart();

       if (!isStopRequested())
       {
           //Close hand and move arm up to prevent robot from crashing
           closeHand();
           armMove(0.75, 2400);

           //strafe left, and then linearDrive to medium junction. then let go of cone
           drive.followTrajectorySequence(trajSeq1);

           //move back, strafe right, and move arm down to appropriate level
           drive.followTrajectorySequence(trajSeq2);

           drive.followTrajectorySequence(trajSeq3);
       }



       //Path Starts Here

       /* OG CODE: WORKS
       Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
               .strafeRight(16)
               .build();

       Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
               .forward(32)
               .build();
       Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
               .strafeRight(8)
               .build();
       Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
               .lineToLinearHeading(new Pose2d(-2,0, Math.toRadians(-90)))
               .build();




       waitForStart();

       if (isStopRequested()) return;

       //actual movement starts here
       closeHand();
       armMove(0.75, 1200);
       drive.followTrajectory(trajectory1);
       drive.followTrajectory(trajectory2);
       openHand();
       drive.followTrajectory(trajectory3);
       drive.followTrajectory(trajectory4);
       */




       Pose2d poseEstimate = drive.getPoseEstimate();
       telemetry.update();

       while (!isStopRequested() && opModeIsActive()) ;
   }


   public void armMove(double speed,
                           double target) {
       int newArmTarget;

       double armPower;

       arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       newArmTarget = arm.getCurrentPosition() + (int)(target);

       // Ensure that the opmode is still active
       if (opModeIsActive()) {

           arm.setTargetPosition(-newArmTarget);
           m1.setPower(0);
           m2.setPower(0);
           m3.setPower(0);
           m4.setPower(0);

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

