package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *   Budget: Mecanum/Holometric Drive
 */

public class HardwareBudgetRobot {

    /* Constants */

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /* Public Opmode Members */
    public DcMotor motor1,motor2, motor3, motor4, arm;
    public CRServo hand;
    BNO055IMU imu;


    public HardwareBudgetRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        /* Drive Hardware */
        motor1 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 1");
        motor2 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 2");
        motor3 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 3");
        motor4 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 4");



        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        /* Intake Hardware */
        arm = myOpMode.hardwareMap.get(DcMotor.class, "Arm");

        hand = myOpMode.hardwareMap.get(CRServo.class, "Hand");
        hand.setPower(0.0);

        /* Gyros */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }


    /* Intake Methods */

}
