package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auton1", group = "Drive Code")
public class Auton1 extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;

    private int lFPos;
    private int rFPos;
    private int lBPos;
    private int rBPos;

    private final double CLICKS_PER_INCH = 0;
    private final double CLICKS_PER_DEGREE = 0;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //************ Dead Reckoning List ************
        //insert auton path here

    }
        private void moveForward(double howFar, double speed) {
            // howFar is in inches
            leftFrontDrive.setTargetPosition(0);
            rightFrontDrive.setTargetPosition(0);
            leftBackDrive.setTargetPosition(0);
            rightBackDrive.setTargetPosition(0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // fetch motor positions
            lFPos = leftFrontDrive.getCurrentPosition();
            rFPos = rightFrontDrive.getCurrentPosition();
            lBPos = leftBackDrive.getCurrentPosition();
            rBPos = rightBackDrive.getCurrentPosition();

            // calculate new targets
            lFPos += (howFar * CLICKS_PER_INCH);
            rFPos += (howFar * CLICKS_PER_INCH);
            lBPos += (howFar * CLICKS_PER_INCH);
            rBPos += (howFar * CLICKS_PER_INCH);

            // move robot to new position
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            leftFrontDrive.setTargetPosition(lFPos);
            rightFrontDrive.setTargetPosition(rFPos);
            leftBackDrive.setTargetPosition(lBPos);
            rightBackDrive.setTargetPosition(rBPos);


            // wait for move to complete
            while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                    leftBackDrive.isBusy() || rightBackDrive.isBusy()) {

                // Display it for the driver.
                telemetry.addLine("Move Forward");
                telemetry.addData("Target", "%7d :%7d", lFPos, rFPos, lBPos, rBPos);
                telemetry.addData("Actual", "%7d :%7d", leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

        }
    }


