package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;
import org.firstinspires.ftc.teamcode.hardware.RoadRunner.drive.SampleMecanumDrive;

@TeleOp(name="Test_TeleOp", group="1")

public class Test_TeleOp extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    Lift lift;
    Intake intake;
    Outtake outtake;
    Horizontal_Lift horlift;
    SampleMecanumDrive drive;
    String mode = "MANUAL";
    boolean outtaketake = false, take = false, peredtake = false, borttake = false;
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        horlift = new Horizontal_Lift(hardwareMap, telemetry);



        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Right Stick Y", gamepad2.right_stick_y);
        //telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.update();
        timer.reset();
        outtake.mayat_down();
        outtake.release();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timerbt = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timerpd = new ElapsedTime();
        timer1.reset();
        timer.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // intake
            if (gamepad1.dpad_up) {
                intake.rotate_mid();
            }
            if (gamepad1.dpad_left) {
                intake.rotate_left();
            }
            if (gamepad1.dpad_right) {
                intake.rotate_right();
            }

            if (gamepad2.a || gamepad1.right_bumper) {
                take = true;
                timer1.reset();
                intake.setsample_take();
                intake.open();
            }
            if (take && timer1.milliseconds() > 100) {
                intake.close();
            }
            if (take && timer1.milliseconds() > 500) {
                intake.setmidpovishe_take();
                take = false;
            }
            if (gamepad2.b) {
                intake.setmid_take();
                intake.open();
            }
            if (gamepad2.x) {
                timerpd.reset();
                peredtake = true;
                intake.open_chut();
            }
            if(peredtake && timerpd.milliseconds() > 200){
                intake.rotate_mid();
                intake.setperedacha();
            }
            if(peredtake && timerpd.milliseconds() > 500){
                intake.close();
                peredtake = false;
            }


            if (gamepad2.right_bumper) {
                horlift.open();
                intake.setmidpovishe_take();
                intake.open();
            }
            //horizontal lift
            if (gamepad2.left_bumper) {
                horlift.close();
            }



            //outtake
            if (gamepad2.dpad_up) {
                outtaketake = true;
                outtake.setPered_take();
                outtake.mayat_up();
                timer.reset();
            }
            if (outtaketake && timer.milliseconds() > 250) {
                outtake.grab();
            }
            if (outtaketake && timer.milliseconds() > 800) {
                intake.setmid_take();
                intake.open();
                outtake.mayat_mid();
            }
            if (outtaketake && timer.milliseconds() > 800) {
                outtake.sethb();
                outtaketake = false;
            }
            if (gamepad1.a) {
                outtake.release();
                outtake.mayat_down();
                outtake.setZad_take();
            }
            if (gamepad1.y) {
                timerbt.reset();
                borttake = true;
                outtake.grab();
            }
            if (borttake && timerbt.milliseconds() > 250) {
                outtake.mayat_up();
                outtake.setspecimen();
                borttake = false;
            }

            if (gamepad2.y || gamepad1.left_bumper) {
                outtake.release();
            }






















            //drivetrain
            if (gamepad1.dpad_down) {
                reset();
            }

            robot_centric();

            //lift
            if (mode == "PID") {
                if (gamepad2.right_stick_y != 0) {
                    mode = "MANUAL";
                }
                lift.update_pid();
            }
            else {
                if (gamepad2.left_stick_button) {
                    mode = "PID";
                    lift.set_target_positiontohigh();
                    outtake.mayat_mid();
                }
                lift.setpower1(gamepad2.right_stick_y);
            }








        }
    }








    public void reset() {
        drive.setPoseEstimate(drive.getPoseEstimate());
    }
    public void field_centric() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }
    public void robot_centric() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        );

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }
}