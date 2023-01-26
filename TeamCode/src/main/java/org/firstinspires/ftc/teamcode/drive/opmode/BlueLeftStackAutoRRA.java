package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.aoe.stem.AOEColorPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BlueLeftStackAutoRRA extends LinearOpMode {

    private Servo _4barServo;
    private CRServo intakeServo;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    OpenCvWebcam webcam;
    String what = "";

    private void Encoder_SlideRaise(double LeftslidePos, double RightslidePos, double LeftslideSpeed, double RightslideSpeed) {
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setTargetPosition((int) (-LeftslidePos / 0.01366433269));
        rightSlide.setTargetPosition((int) (RightslidePos / 0.01366433269));
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(LeftslideSpeed);
        rightSlide.setPower(RightslideSpeed);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        int parkingSpot;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "parkingCam"), cameraMonitorViewId);
        AOEColorPipeline aoepipeline = new AOEColorPipeline("2022-2023", webcam,145,170,25,25);
        webcam.setPipeline(aoepipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        _4barServo = hardwareMap.get(Servo.class, "4barServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");




        Trajectory trajec1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(18)
                .build();

        Trajectory trajec2 = drive.trajectoryBuilder(trajec1.end())
                .forward(6.25)
                .build();

        Trajectory trajec3 = drive.trajectoryBuilder(trajec2.end())
                        .back(6.25)
                                .build();

        Trajectory trajec4 = drive.trajectoryBuilder(trajec3.end())
                        .strafeLeft(40)
                                .build();

        Trajectory goingtostackforward = drive.trajectoryBuilder(trajec4.end())
                        .back(32.25)
                                .build();

        Trajectory linetolinearheadingformediumjunc = drive.trajectoryBuilder(goingtostackforward.end())
                .lineToLinearHeading(new Pose2d(12,57,Math.toRadians(-90)))
                .build();

        Trajectory moveforwardwhenlinetolinearreachesmediumjunction = drive.trajectoryBuilder(linetolinearheadingformediumjunc.end())
                        .forward(5)
                                .build();

        Trajectory movebackatmedium = drive.trajectoryBuilder(moveforwardwhenlinetolinearreachesmediumjunction.end())
                .back(6)
                .build();

        Trajectory parkingLocationthree = drive.trajectoryBuilder(movebackatmedium.end())
                        .strafeLeft(12)
                                .build();

        Trajectory parkingLocationtwo = drive.trajectoryBuilder(new Pose2d(12,57,Math.toRadians(-90)))
                        .strafeRight(13.5)
                                .build();

        Trajectory parkingLocationone = drive.trajectoryBuilder(new Pose2d(12,57,Math.toRadians(-90)))
                        .strafeRight(33.5)
                                .build();










        waitForStart();

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;
        parkingSpot = 0;
        what = aoepipeline.ColorValue;

        telemetry.addData("What", aoepipeline.ColorValue);
        telemetry.update();
        sleep(100);

        if (what == "blue") {
            parkingSpot = 1;
        } else if (what == "green") {
            parkingSpot = 2;
        } else if (what == "red") {
            parkingSpot = 3;

        }

        sleep(1000);

        intakeServo.setPower(1);
        _4barServo.setPosition(0.5);
        sleep(20);
        drive.followTrajectory(trajec1);
        Encoder_SlideRaise(8.5, 8.5, 0.5, 0.5);
        drive.followTrajectory(trajec2);
        _4barServo.setPosition(0.8);
        sleep(500);
        intakeServo.setPower(-1);
        sleep(500);
        _4barServo.setPosition(0.385);
        sleep(20);
        intakeServo.setPower(0);
        drive.followTrajectory(trajec3);
        Encoder_SlideRaise(4.5,4.5,0.5,0.5);
        drive.followTrajectory(trajec4);
        drive.followTrajectory(goingtostackforward);
        Encoder_SlideRaise(1.5,1.5,0.5,0.5);
        intakeServo.setPower(1);
        sleep(995);
        Encoder_SlideRaise(22.5,22.5,0.5,0.5);
        _4barServo.setPosition(0.5);
        sleep(500);
        drive.followTrajectory(linetolinearheadingformediumjunc);
        drive.followTrajectory(moveforwardwhenlinetolinearreachesmediumjunction);
        _4barServo.setPosition(0.8);
        intakeServo.setPower(-1);
        sleep(500);
        drive.followTrajectory(movebackatmedium);
        _4barServo.setPosition(0.5);
        sleep(200);
        intakeServo.setPower(0);
        Encoder_SlideRaise(0,0,0.5,0.5);

        if (parkingSpot == 1) {
            drive.followTrajectory(parkingLocationone);

        } else if (parkingSpot == 2) {

            drive.followTrajectory(parkingLocationtwo);



        } else if (parkingSpot == 3) {

            drive.followTrajectory(parkingLocationthree);



        }








        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();



        while (!isStopRequested() && opModeIsActive()) ;

    }
}


