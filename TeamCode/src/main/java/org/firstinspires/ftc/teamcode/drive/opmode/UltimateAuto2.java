package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.AuxHardwareMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class UltimateAuto2 extends LinearOpMode {
    AuxHardwareMap robot = new AuxHardwareMap();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        robot.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory shoot = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(63, 20, Math.toRadians(180)))
                .build();
        Trajectory wobbleSpot = drive.trajectoryBuilder(shoot.end())
                .lineToConstantHeading(new Vector2d(64, -10))
                .build();
        Trajectory wobble2 = drive.trajectoryBuilder(wobbleSpot.end())
                .lineToSplineHeading(new Pose2d(32, 22, Math.toRadians(0)))
                .build();
        Trajectory wobbleDeliver = drive.trajectoryBuilder(wobble2.end())
                .lineToSplineHeading(new Pose2d(54, -16, Math.toRadians(180)))
                .build();
        Trajectory park = drive.trajectoryBuilder(wobble2.end(), true)
                .lineToSplineHeading(new Pose2d(75, 22, Math.toRadians(360)))
                .build();

        robot.belt.setPower(.5);

        drive.followTrajectory(shoot);
        shoot(4);
        drive.followTrajectory(wobbleSpot);
        dropWobble();
        drive.followTrajectory(wobble2);
        grabWobble();
        drive.followTrajectory(wobbleDeliver);
        dropWobble();
        liftArm();
        drive.followTrajectory(park);


        sleep(2000);


    }


    public void shoot(double shots) {
        ((DcMotorEx) robot.shooter).setVelocity(1650);
        sleep(2000);
        int i;
        for (i = 0; i < shots; i++) {
            robot.shooter_Servo.setPosition(0.57);
            sleep(300);
            robot.shooter_Servo.setPosition(0.15);
            sleep(300);
        }
        ((DcMotorEx) robot.shooter).setVelocity(0);

    }


    public void liftArm() {
        robot.wobble.setTargetPosition(10);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.wobble.setPower(Math.abs(.8));
        while (opModeIsActive() && (robot.wobble.isBusy())) {


        }

        // Stop all motion;
        robot.wobble.setPower(0);
    }

    public void lowerWobble() {
        robot.wobble.setTargetPosition(-1165);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.wobble.setPower(Math.abs(.5));

        while (opModeIsActive() && (robot.wobble.isBusy())) {
        }

        // Stop all motion;
        robot.wobble.setPower(0);
    }

    public void grabWobble() {

        robot.wobbleservo1.setPosition(.5);
        robot.wobbleservo2.setPosition(.5);
        sleep(500);
        robot.wobble.setTargetPosition(10);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.wobble.setPower(Math.abs(.8));
        while (opModeIsActive() && (robot.wobble.isBusy())) {


        }

        // Stop all motion;
        robot.wobble.setPower(0);


    }


    public void dropWobble() {
        robot.wobble.setTargetPosition(-1265);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.wobble.setPower(Math.abs(.5));
        while (opModeIsActive() && (robot.wobble.isBusy())) {
        }

        // Stop all motion;
        robot.wobble.setPower(0);
        robot.wobbleservo1.setPosition(1);
        robot.wobbleservo2.setPosition(0);
        sleep(500);


    }


}
