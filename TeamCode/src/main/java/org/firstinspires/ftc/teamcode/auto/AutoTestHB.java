package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;
import org.firstinspires.ftc.teamcode.hardware.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.RoadRunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "HB", group = "test")
public class AutoTestHB extends LinearOpMode {
    public static int specup = 650, specup2=1200;
    Intake intake;
    Outtake outtake;
    Lift lift;
    Horizontal_Lift horlift;
    SampleMecanumDrive drive;
    Pose2d startpose = new Pose2d(0, 0, 0);
    public static Pose2d P1 = new Pose2d(5, 43, Math.toRadians(-45));
    public static Pose2d P2 = new Pose2d(13.7, 28.8, Math.toRadians(0));
    public static Pose2d P3 = new Pose2d(6, 43, Math.toRadians(-60));
    public static Pose2d P4 = new Pose2d(14.5, 41, Math.toRadians(0));
    public static Pose2d P5 = new Pose2d(6, 43, Math.toRadians(-60));
    public static Pose2d P6 = new Pose2d(20, 32.5, Math.toRadians(38.3));
    public static Pose2d P7 = new Pose2d(6, 43, Math.toRadians(-63));
    public static Pose2d p = new Pose2d();
    enum State {
        P_1,align,A1,A2,A3,A4,A5,A6,A7,
        P_2,
        P_3,
        P_6, P_5,
        P_4,
        IDLE
    }
    @Override
    public void runOpMode() throws InterruptedException {
        State currentState = State.IDLE;


        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        horlift = new Horizontal_Lift(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        intake.rotate_mid();
        intake.close();
        intake.setperedacha();
        horlift.close();
        outtake.grab();

        Trajectory tr1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(P1,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        Trajectory tr2 = drive.trajectoryBuilder(tr1.end())
                .lineToLinearHeading(P2,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        Trajectory tr3 = drive.trajectoryBuilder(tr2.end())
                .addTemporalMarker(0, () -> {
                    horlift.close();
                    outtake.setPered_take();
                    intake.open_chut();
                    sleep(200);
                    intake.rotate_mid();
                    intake.setperedacha();
                    sleep(300);
                    intake.close();
                    sleep(500);
                    outtake.mayat_up1();
                    outtake.grab();
                    sleep(300);
                    intake.setmid_take();
                    intake.open();

                })
                .lineToLinearHeading(P3,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .build();
        Trajectory tr4 = drive.trajectoryBuilder(tr3.end())
                .lineToLinearHeading(P4,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        Trajectory tr5 = drive.trajectoryBuilder(tr4.end())
                .addTemporalMarker(0, () -> {
                    horlift.close();
                    outtake.setPered_take();
                    intake.open_chut();
                    sleep(200);
                    intake.rotate_mid();
                    intake.setperedacha();
                    sleep(300);
                    intake.close();
                    sleep(500);
                    outtake.mayat_up1();
                    outtake.grab();
                    sleep(300);
                    intake.setmid_take();
                    intake.open();
                })
                .lineToLinearHeading(P5,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .build();

        Trajectory tr6 = drive.trajectoryBuilder(tr5.end())
                .lineToLinearHeading(P6,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        Trajectory tr7 = drive.trajectoryBuilder(tr6.end())
                .addTemporalMarker(0, () -> {
                    horlift.close();
                    outtake.setPered_take();
                    intake.open_chut();
                    sleep(200);
                    intake.rotate_mid();
                    intake.setperedacha();
                    sleep(300);
                    intake.close();
                    sleep(500);
                    outtake.mayat_up1();
                    outtake.grab();
                    sleep(300);
                    intake.setmid_take();
                    intake.open();
                })
                .lineToLinearHeading(P7,
                        SampleMecanumDrive.getVelocityConstraint(30, Math.PI*2/3,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .build();




        waitForStart();
        outtake.sethb();
        lift.set_to_high_basket();
        currentState = State.A1;
        drive.followTrajectoryAsync(tr1);
        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()) {
            switch (currentState) {
                case A1:
                    if (!drive.isBusy()) {
                        sleep(300);
                        outtake.sethb();
                        sleep(700);
                        outtake.release();
                        sleep(200);
                        outtake.setPered_take();
                        currentState = State.A2;
                        timer.reset();
                        drive.followTrajectoryAsync(tr2);

                    }
                    break;
                case A2:
                    outtake.setPered_take();
                    if(timer.milliseconds() > 300)
                        lift.set_target_position(0);
                    if (!drive.isBusy()) {
                        horlift.open();
                        intake.open();
                        intake.setmidpovishe_take();
                        sleep(300);
                        intake.setsample_take();
                        sleep(300);
                        intake.close();
                        sleep(200);
                        currentState = State.A3;
                        drive.followTrajectoryAsync(tr3);
                    }
                    break;
                case A3:

                    if (!drive.isBusy()) {
                        ElapsedTime t = new ElapsedTime();
                        t.reset();
                        lift.set_to_high_basket();
                        while(t.milliseconds() < 1300)
                            lift.update_pid();
                        lift.hold_position();
                        sleep(300);
                        outtake.sethb();
                        sleep(1000);
                        outtake.release();
                        sleep(200);
                        timer.reset();
                        drive.followTrajectoryAsync(tr4);
                        currentState = State.A4;
                    }
                    break;
                case A4:
                    outtake.setPered_take();
                    if(timer.milliseconds() > 300)
                        lift.set_target_position(0);
                    if (!drive.isBusy()) {
                        horlift.open();
                        intake.open();
                        intake.setmidpovishe_take();
                        sleep(300);
                        intake.setsample_take();
                        sleep(300);
                        intake.close();
                        sleep(200);
                        currentState = State.A5;
                        drive.followTrajectoryAsync(tr3);
                    }
                    break;
                case A5:
                    if (!drive.isBusy()) {
                        ElapsedTime t = new ElapsedTime();
                        t.reset();
                        lift.set_to_high_basket();
                        while(t.milliseconds() < 1300)
                            lift.update_pid();
                        lift.hold_position();
                        sleep(300);
                        outtake.sethb();
                        sleep(1000);
                        outtake.release();
                        sleep(200);
                        timer.reset();
                        drive.followTrajectoryAsync(tr6);
                        currentState = State.A6;
                    }
                    break;
                case A6:
                    outtake.setPered_take();
                    if(timer.milliseconds() > 300)
                        lift.set_target_position(0);
                    if (!drive.isBusy()) {
                        horlift.open();
                        intake.open();
                        intake.rotate_right();
                        intake.setmidpovishe_take();
                        sleep(500);
                        intake.setsample_take();
                        sleep(300);
                        intake.close();
                        sleep(200);
                        drive.followTrajectoryAsync(tr7);
                        currentState = State.A7;
                    }
                    break;
                case A7:

                    if (!drive.isBusy()) {
                        ElapsedTime t = new ElapsedTime();
                        t.reset();
                        lift.set_to_high_basket();
                        while(t.milliseconds() < 1300)
                            lift.update_pid();
                        lift.hold_position();
                        sleep(300);
                        outtake.sethb();
                        sleep(1000);
                        outtake.release();
                        sleep(200);
                        outtake.setPered_take();
                        intake.setmidpovishe_take();
                        timer.reset();
                        currentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    outtake.setPered_take();
                    if(timer.milliseconds() > 300)
                        lift.set_target_position(0);
                    break;
            }
            drive.update();
            lift.update_pid();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("State:", currentState);
            telemetry.update();
        }

    }

}