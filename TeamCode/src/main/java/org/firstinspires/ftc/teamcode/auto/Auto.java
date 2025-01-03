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
@Autonomous(name = "Auto 4 specimen")
public class Auto extends LinearOpMode {
    public static int specup = 640, specup2=1200;
    Intake intake;
    Outtake outtake;
    Lift lift;
    Horizontal_Lift horlift;
    SampleMecanumDrive drive;
    Pose2d startpose = new Pose2d(0, 0, 0);
    public static Pose2d P1 = new Pose2d(32, 0, 0);
    public static Pose2d P2 = new Pose2d(19-0.9, -27+0.6, Math.toRadians(321.7));
    public static Pose2d P3 = new Pose2d(19.6-0.7, -35.5+2, Math.toRadians(321.7));
    public static Pose2d P4 = new Pose2d(19-2.7, -46.19+0.3, Math.toRadians(321.7));
    public static Pose2d P5 = new Pose2d(18.59, -29.3, Math.toRadians(210.416));
    public static Pose2d P6 = new Pose2d(0, -29.3, 0);
    public static Pose2d p = new Pose2d();
    enum State {
        P_1,align,A1,A2,A3,A4,A5,A6,A7,SP1, R1, SP2, R2, SP3, R3, SP4, R, R4,
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
                .addTemporalMarker(0, () -> {
                    outtake.setspecimen();

                })
                .lineToLinearHeading(P1,
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        Trajectory tr2 = drive.trajectoryBuilder(tr1.end())
                .addTemporalMarker(0, () -> {
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    lift.set_target_position(specup2);
                    while(timer.milliseconds() < 600) {
                        lift.update_pid();
                    }
                    outtake.release();

                })
                .addTemporalMarker(0.8, () -> {
                    lift.set_target_position(0);
                    outtake.setZad_take();
                })
                .lineToLinearHeading(P2,
                        SampleMecanumDrive.getVelocityConstraint(60, 3.5,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .addDisplacementMarker(() -> {
                    horlift.open();
                    intake.setmidpovishe_take();
                    intake.open();
                })
                .build();

        Trajectory tr3 = drive.trajectoryBuilder(tr2.end())
                .addTemporalMarker(0, () -> {
                    intake.setsample_take();
                    intake.open();
                    sleep(100);
                    intake.close();
                    sleep(200);
                })
                .lineToLinearHeading(P5,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))

                .build();

        Trajectory tr4 = drive.trajectoryBuilder(tr3.end())
                .lineToLinearHeading(P3,
                        SampleMecanumDrive.getVelocityConstraint(60, 3.5,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        Trajectory tr5 = drive.trajectoryBuilder(tr4.end())
                .addTemporalMarker(0, () -> {
                    intake.setsample_take();
                    intake.open();
                    sleep(100);
                    intake.close();
                    sleep(200);
                })
                .lineToLinearHeading(P5,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))

                .build();
        Trajectory tr6 = drive.trajectoryBuilder(tr5.end())
                .lineToLinearHeading(P4,
                        SampleMecanumDrive.getVelocityConstraint(60, 3.5,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        Trajectory tr7 = drive.trajectoryBuilder(tr6.end())
                .addTemporalMarker(0, () -> {
                    intake.setsample_take();
                    intake.open();
                    sleep(100);
                    intake.close();
                    sleep(200);
                })
                .lineToLinearHeading(P5,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))

                .build();
        Trajectory r = drive.trajectoryBuilder(tr7.end())
                .addTemporalMarker(0, () -> {
                    horlift.close();
                    intake.setperedacha();
                    intake.close();
                    outtake.setZad_take();
                    outtake.release();
                })
                .lineToLinearHeading(new Pose2d(P6.getX()-1.5, P6.getY()-5, 0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))
                .build();
        Trajectory specimen1 = drive.trajectoryBuilder(r.end())
                .addTemporalMarker(0, () -> {
                    sleep(200);
                    outtake.grab();
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_target_position(specup);
                })
                .splineToLinearHeading(new Pose2d(P1.getX()+1.6, P1.getY()+3, P1.getHeading()), 0,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))

                .build();
        Trajectory r1 = drive.trajectoryBuilder(specimen1.end(), true)

                .addTemporalMarker(0, () -> {
                    outtake.setZad_take();
                    outtake.release();
                    lift.set_target_position(0);
                })

                .splineToLinearHeading(new Pose2d(P6.getX()-3, P6.getY()-7, 0), Math.PI,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))
                .build();
        Trajectory specimen2 = drive.trajectoryBuilder(r1.end())
                .addTemporalMarker(0, () -> {
                    sleep(200);
                    outtake.grab();
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_target_position(specup);
                })
                .splineToLinearHeading(new Pose2d(P1.getX()+2.1, P1.getY()+1.5, P1.getHeading()),0.3,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))
                .addDisplacementMarker(() -> {

                })
                .build();
        Trajectory r2 = drive.trajectoryBuilder(specimen2.end(), true)

                .addTemporalMarker(0, () -> {
                    outtake.setZad_take();
                    outtake.release();
                    lift.set_target_position(0);
                })

                .splineToLinearHeading(new Pose2d(P6.getX()-4, P6.getY()-7, -Math.toRadians(10)), Math.PI,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))
                .build();
        Trajectory specimen3 = drive.trajectoryBuilder(r2.end())
                .addTemporalMarker(0, () -> {
                    sleep(200);
                    outtake.grab();
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_target_position(specup);
                })
                .splineToLinearHeading(new Pose2d(P1.getX()+3.2, P1.getY()+6, -Math.toRadians(10)),0.3,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))
                .addDisplacementMarker(() -> {

                })
                .build();
        Trajectory r3 = drive.trajectoryBuilder(specimen3.end(), true)

                .addTemporalMarker(0, () -> {
                    outtake.setZad_take();
                    outtake.release();
                    lift.set_target_position(0);
                })

                .splineToLinearHeading(new Pose2d(P6.getX()-4, P6.getY()-7, 0), Math.PI,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(37))
                .build();
        waitForStart();
        lift.set_target_position(specup);
        currentState = State.A1;
        drive.followTrajectoryAsync(tr1);
        while(opModeIsActive()) {
            switch (currentState) {
                case A1:
                    if (!drive.isBusy()) {
                        currentState = State.A2;
                        drive.followTrajectoryAsync(tr2);
                    }
                    break;
                case A2:
                    if (!drive.isBusy()) {
                        currentState = State.A3;
                        drive.followTrajectoryAsync(tr3);
                    }
                    break;
                case A3:

                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(tr4);
                        currentState = State.A4;
                    }
                    break;
                case A4:
                    horlift.open();
                    intake.open();
                    intake.setmidpovishe_take();
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(tr5);
                        currentState = State.A5;
                    }
                    break;
                case A5:

                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(tr6);
                        currentState = State.A6;
                    }
                    break;
                case A6:
                    horlift.open();
                    intake.open();
                    intake.setmidpovishe_take();
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(tr7);
                        currentState = State.A7;
                    }
                    break;
                case A7: // бросил 3 сэмпл к хюмену

                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(r);
                        currentState = State.R;
                    }
                    break;
                case R:
                    horlift.open();
                    intake.open();
                    horlift.close();
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(specimen1);
                        currentState = State.SP1;
                    }
                    break;
                case SP1:
                    if (!drive.isBusy()) {
                        ElapsedTime timer = new ElapsedTime();
                        timer.reset();
                        lift.set_target_position(specup2);
                        while(timer.milliseconds() < 600) {
                            lift.update_pid();
                        }
                        outtake.release();
                        drive.followTrajectoryAsync(r1);
                        currentState = State.R1;
                    }
                    break;
                case R1:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(specimen2);
                        currentState = State.SP2;
                    }
                    break;
                case SP2:
                    if (!drive.isBusy()) {
                        ElapsedTime timer = new ElapsedTime();
                        timer.reset();
                        lift.set_target_position(specup2);
                        while(timer.milliseconds() < 600) {
                            lift.update_pid();
                        }
                        outtake.release();
                        drive.followTrajectoryAsync(r2);
                        currentState = State.R2;
                    }
                    break;
                case R2:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(specimen3);
                        currentState = State.SP3;
                    }
                    break;
                case SP3:
                    if (!drive.isBusy()) {
                        ElapsedTime timer = new ElapsedTime();
                        timer.reset();
                        lift.set_target_position(specup2);
                        while(timer.milliseconds() < 600) {
                            lift.update_pid();
                        }
                        outtake.release();
                        drive.followTrajectoryAsync(r3);
                        currentState = State.R3;
                    }
                    break;
                case R3:
                    intake.setmidpovishe_take();
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:

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