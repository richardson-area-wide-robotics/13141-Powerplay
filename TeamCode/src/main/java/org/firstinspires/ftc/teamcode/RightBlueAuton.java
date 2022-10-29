package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RightBlueAuton", group = "Drive Code")
public class RightBlueAuton extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    //private DcMotor leftArmDrive = null;
    //private DcMotor rightArmDrive = null;
    //private DcMotor leftIntakeDrive = null;
    //private DcMotor rightIntakeDrive = null;

    private int lFPos;
    private int rFPos;
    private int lBPos;
    private int rBPos;
    private int armRPos;
    private int armLPos;

    private final double CLICKS_PER_INCH = 31.1001805671;
    private final double CLICKS_PER_DEGREE = 0.05499719409;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDemo aprilTagDemo;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);
        String parking = null;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //leftArmDrive = hardwareMap.get(DcMotor.class, "left_arm_drive");
        //rightArmDrive = hardwareMap.get(DcMotor.class, "right_arm_drive");
        //leftIntakeDrive = hardwareMap.get(DcMotor.class, "left_intake_drive");
        //rightIntakeDrive = hardwareMap.get(DcMotor.class, "right_intake_drive");

        aprilTagDemo = new AprilTagDemo();

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftArmDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightArmDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftIntakeDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightIntakeDrive.setDirection(DcMotor.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // TODO
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // TODO
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            //
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                telemetry.addData("Tag Count", detections.size());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

                    }

                }

                telemetry.update();

                if(detections.size()>=1) {
                    telemetry.addLine(detections.size() + "  targets found");
                    if (detections.get(0).id == 0) {
                        telemetry.addLine("Target is Left Parking");
                        parking = "left";
                        break;
                    } else if (detections.get(0).id == 1) {
                        telemetry.addLine("Target is Middle Parking");
                        parking = "middle";
                        break;
                    } else if(detections.get(0).id == 2){
                        telemetry.addLine("Target is Right Parking");
                        parking = "right";
                        break;
                    } else{
                        telemetry.addLine("Target is not 1 2 or 3");
                    }
                }else{
                    telemetry.addLine("no targets found");
                }

            }

            sleep(20);
        }


//************ Dead Reckoning List ************
        //aprilTagDemo.detect();
        if (parking == "left") {
            moveForward(15, 1);
            telemetry.addLine("Moving forward 15");
            turnClockwise(-90, 1);
            telemetry.addLine("Turning Counter Clockwise 90Deg");
            moveForward(12, 1);
            telemetry.addLine("Moving forward 12");
        } else if (parking == "middle") {
            moveForward(15, 1);
            telemetry.addLine("Moving forward 15");
        } else if (parking == "right") {
            moveForward(15, 1);
            telemetry.addLine("Moving forward 15");
            turnClockwise(90, 1);
            telemetry.addLine("Turning Clockwise 90Deg");
            moveForward(12, 1);
            telemetry.addLine("Moving forward 12");
        }


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

    private void turnClockwise(double whatAngle, double speed) {
        //whatAngle is in degrees(positive means turn clockwise, negative means turn counterclockwise

        // fetch motor positions
        lFPos = leftFrontDrive.getCurrentPosition();
        rFPos = rightFrontDrive.getCurrentPosition();
        lBPos = leftBackDrive.getCurrentPosition();
        rBPos = rightBackDrive.getCurrentPosition();

        // calculate new targets
        lFPos += whatAngle * CLICKS_PER_DEGREE;
        rFPos -= whatAngle * CLICKS_PER_DEGREE;
        lBPos += whatAngle * CLICKS_PER_DEGREE;
        rBPos -= whatAngle * CLICKS_PER_DEGREE;

        // move robot to new position
        leftFrontDrive.setTargetPosition(lFPos);
        rightFrontDrive.setTargetPosition(rFPos);
        leftBackDrive.setTargetPosition(lBPos);
        rightBackDrive.setTargetPosition(rBPos);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // wait for move to complete
        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy() || rightBackDrive.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
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



    private int getConeID() {
        return this.aprilTagDemo.getConeID();
        }

    }