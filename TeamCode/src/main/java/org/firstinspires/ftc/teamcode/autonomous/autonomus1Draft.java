package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(group = "drive")
public class autonomus1Draft extends LinearOpMode {
    private SampleMecanumDrive drive;

    //enumerates mode
    enum Mode {
        AUTO_MODE,
        TELEOP_MODE
    }
    private autonomus1Draft.Mode mode;

    @Override
    public void runOpMode(){

        drive = new SampleMecanumDrive(hardwareMap);
        //sets the current mode as autonomous
        mode = Mode.AUTO_MODE;

        //builds my trajectory
        Trajectory mySpline = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        //declares a clock, which will be used to keep track of autonomous duration
        NanoClock clock = NanoClock.system();

        //waits until the play button is pressed
        waitForStart();

        //finalize runOpMode thread if stop is requested
        if (isStopRequested()) return;
        //sets the OpModes startTime
        double timeStart = clock.seconds();

        while (!isStopRequested()){

            //when 29 seconds have passed OR driveTrain driver presses start, switch to TeleOp Mode
            if(clock.seconds() - timeStart > 29.0 ){
               mode = Mode.TELEOP_MODE;
            }

            telemetry.addData("Mode:", mode);

            switch (mode){

                case AUTO_MODE:
                    //follow trajectory, needs testing
                    drive.followTrajectory(mySpline);
                    break;

                case TELEOP_MODE:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();

                    Pose2d poseEstimate = drive.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    break;
            }


            telemetry.update();
        }
    }
}
