package org.firstinspires.ftc.teamcode.drive.opmode;

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous
public class BlueLeftStackAutoRRA extends LinearOpMode {

    private Servo _4barServo;
    private CRServo intakeServo;
    private DcMotor leftSlide;
    private DcMotor rightSlide;

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
                        .strafeLeft(38.5)
                                .build();

        Trajectory goingtostackforward = drive.trajectoryBuilder(trajec4.end())
                        .back(30)
                                .build();



        waitForStart();

        if (isStopRequested()) return;

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
        _4barServo.setPosition(0.25);
        sleep(20);
        intakeServo.setPower(0);
        drive.followTrajectory(trajec3);
        Encoder_SlideRaise(2.5,2.5,0.5,0.5);
        drive.followTrajectory(trajec4);
        drive.followTrajectory(goingtostackforward);


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();



        while (!isStopRequested() && opModeIsActive()) ;
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}


