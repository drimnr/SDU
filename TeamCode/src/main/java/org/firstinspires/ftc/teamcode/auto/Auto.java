package org.firstinspires.ftc.teamcode.auto;/*package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
@Autonomous(group = "test")
public class Auto extends LinearOpMode {
    public static int specup = 1100, specdown = 600;
    Intake intake;
    Outtake outtake;
    Lift lift;
    Horizontal_Lift horlift;
    SampleMecanumDrive drive;
    Pose2d startpose = new Pose2d(0, 0, 0);
    public static Pose2d A1 = new Pose2d(-23.53, -9.27, Math.toRadians(0));
    public static Pose2d A2 = new Pose2d(-19.63, 23.33, Math.toRadians(147.00));
    public static Pose2d A3 = new Pose2d(-14.47, 25.0, Math.toRadians(41.00));
    public static Pose2d A4 = new Pose2d(-10.00, 14.26, Math.toRadians(49.00));
    public static Pose2d A22 = new Pose2d(-20.63, 37.42, Math.toRadians(147.00));
    public static Pose2d p = new Pose2d(-11.17, 30.3, Math.toRadians(0));
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        horlift = new Horizontal_Lift(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        intake.mayat_up();
        intake.rotate_mid();
        intake.open();
        outtake.mayat_down();
        sleep(1000);
        outtake.grab();


        Trajectory tr1 = drive.trajectoryBuilder(startpose)
                .addTemporalMarker(0, () -> {
                    outtake.mayat_up();
                    lift.set_target_position(specup);
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        lift.update_pid();
                    }
                    lift.hold_position();
                })
                .lineToLinearHeading(A1,
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .build();


        Trajectory tr2 = drive.trajectoryBuilder(tr1.end())
                .lineToLinearHeading(new Pose2d(A2.getX() - 2.2, A2.getY(), A2.getHeading()))
                .build();
        Trajectory tr3 = drive.trajectoryBuilder(tr2.end())
                .lineToLinearHeading(A3)
                .build();
        Trajectory tr4 = drive.trajectoryBuilder(tr3.end())
                .lineToLinearHeading(A4,
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();

        Trajectory tr5 = drive.trajectoryBuilder(tr4.end())
                .addTemporalMarker(0, () -> {
                    outtake.mayat_up();
                    lift.set_target_position(specup);
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        lift.update_pid();
                    }
                    lift.hold_position();
                })
                .lineToLinearHeading(A1,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();



        waitForStart();


        drive.followTrajectory(tr1);


        lift.set_target_position(specdown);
        ElapsedTime timer41 = new ElapsedTime();
        timer41.reset();
        while (timer41.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();
        outtake.release();
        sleep(1000);


        outtake.mayat_down();
        horlift.open();
        intake.mayat_mid2();
        lift.set_target_position(0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();
        intake.mayat_mid2();


        drive.followTrajectory(tr2);
        intake.rotate_left();

        intake.mayat_down();
        sleep(400);
        intake.close();
        sleep(400);
        intake.mayat_mid();
        drive.followTrajectory(tr3);



        intake.open();
        sleep(200);
        intake.mayat_mid();
        intake.rotate_mid();

        drive.followTrajectory(tr4);
        intake.rotate_mid();
        intake.mayat_down();
        sleep(1000);
        intake.close();
        sleep(700);
        intake.mayat_up();
        horlift.close();
        sleep(1000);

        intake.open_chut();
        sleep(250);
        outtake.grab();
        sleep(550);
        intake.mayat_mid();
        intake.open();
        outtake.mayat_up();

        drive.followTrajectory(tr5);
        lift.set_target_position(specdown);
        ElapsedTime timer2 = new ElapsedTime();
        timer2.reset();
        while (timer2.milliseconds() < 2000) {
            lift.update_pid();
        }
        lift.hold_position();
        outtake.release();
        sleep(1000);



        Trajectory tr22 = drive.trajectoryBuilder(tr1.end())

                .lineToLinearHeading(new Pose2d(A4.getX() - 2, A4.getY()+2, A4.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .build();

        Trajectory tr32 = drive.trajectoryBuilder(tr22.end())

                .lineToLinearHeading(A1,
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .build();

        outtake.mayat_down();
        horlift.open();
        intake.mayat_mid2();
        lift.set_target_position(0);
        ElapsedTime timer7 = new ElapsedTime();
        timer7.reset();
        while (timer7.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();
        drive.followTrajectory(tr22);
        intake.rotate_mid();
        intake.mayat_down();
        sleep(1000);
        intake.close();
        sleep(700);
        intake.mayat_up();
        horlift.close();
        sleep(1000);

        intake.open_chut();
        sleep(250);
        outtake.grab();
        sleep(550);
        intake.mayat_mid();
        intake.open();
        outtake.mayat_up();



        outtake.mayat_up();
        lift.set_target_position(specup);
        ElapsedTime timer4 = new ElapsedTime();
        timer4.reset();
        while (timer4.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();

        drive.followTrajectory(tr32);

        lift.set_target_position(specdown);
        ElapsedTime timer71 = new ElapsedTime();
        timer71.reset();
        while (timer71.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();
        outtake.release();
        sleep(1000);


        outtake.mayat_down();
        intake.mayat_mid2();
        lift.set_target_position(0);
        ElapsedTime timer77 = new ElapsedTime();
        timer77.reset();
        while (timer77.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();




        Trajectory park = drive.trajectoryBuilder(tr5.end())
                .lineToLinearHeading(p)
                .build();
        drive.followTrajectory(park);
        /*


        Trajectory tr22 = drive.trajectoryBuilder(tr1.end())
                .addTemporalMarker(0, () -> {
                    outtake.mayat_down();
                    horlift.open();
                    intake.mayat_mid2();
                    lift.set_target_position(0);
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    while (timer.milliseconds() < 1000) {
                        lift.update_pid();
                    }
                    lift.hold_position();
                })
                .lineToLinearHeading(A3)
                .build();

        Trajectory tr32 = drive.trajectoryBuilder(tr22.end())
                .lineToLinearHeading(A3)
                .build();


        drive.followTrajectory(tr22);
        intake.rotate_left();

        intake.mayat_down();
        sleep(400);
        intake.close();
        sleep(400);

        drive.followTrajectory(tr32);



        intake.open();
        sleep(200);
        intake.mayat_up();


        drive.followTrajectory(tr4);
        intake.rotate_mid();
        intake.mayat_down();
        sleep(1000);
        intake.close();
        sleep(200);
        intake.mayat_up();
        horlift.close();
        sleep(1000);

        intake.open_chut();
        sleep(250);
        outtake.grab();
        sleep(550);
        intake.mayat_mid();
        intake.open();
        outtake.mayat_up();

        drive.followTrajectory(tr5);
        lift.set_target_position(specdown);
        ElapsedTime timer3 = new ElapsedTime();
        timer1.reset();
        while (timer3.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();
        outtake.release();
        sleep(1000);

        outtake.mayat_down();
        horlift.open();
        intake.mayat_mid2();
        lift.set_target_position(0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 1000) {
            lift.update_pid();
        }
        lift.hold_position();

        Trajectory park = drive.trajectoryBuilder(tr5.end())
                .lineToLinearHeading(p)
                .build();


        drive.followTrajectory(park);

         */
  //  }

//}