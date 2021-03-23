package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class TestOpMode extends LinearOpMode {
    private DcMotor shooter = null;
    private Servo shooter_Servo;
    private DcMotor belt = null;
    private DcMotor wobble = null;
    private Servo wobbleservo1;
    private Servo wobbleservo2;
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {


        shooter = hardwareMap.get(DcMotor.class, "flywheel" );
        belt = hardwareMap.get(DcMotor.class, "belt" );
        shooter_Servo = hardwareMap.get(Servo.class, "shooter");
        wobbleservo1 = hardwareMap.get(Servo.class, "wobble1");
        wobbleservo2 = hardwareMap.get(Servo.class, "wobble2");
        wobble = hardwareMap.get(DcMotor.class, "wobble" );


        wobble = hardwareMap.get(DcMotor.class, "wobble" );
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory shoot = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(63, 20, Math.toRadians(180)))
                .build();
        Trajectory wobbleSpot = drive.trajectoryBuilder(shoot.end())
                .lineToConstantHeading(new Vector2d(67, -5))
                .build();
        Trajectory wobble2 = drive.trajectoryBuilder(wobbleSpot.end())
                .lineToSplineHeading(new Pose2d(32, 22, Math.toRadians(0)))
                .build();
        Trajectory wobbleDeliver = drive.trajectoryBuilder(wobble2.end())
                .lineToSplineHeading(new Pose2d(58, -5, Math.toRadians(180)))
                .build();
        Trajectory park = drive.trajectoryBuilder(wobble2.end())
                .lineToSplineHeading(new Pose2d(75, 22, Math.toRadians(360)))
                .build();

        belt.setPower(.5);
        drive.followTrajectory(shoot);
        shoot(4);
        drive.followTrajectory(wobbleSpot);
        dropWobble();
        drive.followTrajectory(wobble2);
        grabWobble();
        drive.followTrajectory(wobbleDeliver);
        dropWobble();
        drive.followTrajectory(park);




        sleep(2000);


    }


    public void shoot(double shots) {
        ((DcMotorEx) shooter).setVelocity(2000);
        sleep(2000);
        int i;
        for (i = 0; i < shots; i++) {
            shooter_Servo.setPosition(0.57);
            sleep(300);
            shooter_Servo.setPosition(0.15);
            sleep(300);
        }
        ((DcMotorEx) shooter).setVelocity(0);

    }

    public void lowerWobble() {
        wobble.setTargetPosition(-1165);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        wobble.setPower(Math.abs(.5));
        while (opModeIsActive() && (wobble.isBusy())) {
        }

        // Stop all motion;
        wobble.setPower(0);
    }
        public void grabWobble() {

            wobbleservo1.setPosition(.5);
            wobbleservo2.setPosition(.5);
            sleep(500);
            wobble.setTargetPosition(10);
            wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            wobble.setPower(Math.abs(.8));
            while (opModeIsActive() && (wobble.isBusy())) {


            }

            // Stop all motion;
            wobble.setPower(0);


        }



    public void dropWobble() {
        wobble.setTargetPosition(-1265);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        wobble.setPower(Math.abs(.5));
        while (opModeIsActive() && (wobble.isBusy())) {
        }

        // Stop all motion;
        wobble.setPower(0);
        wobbleservo1.setPosition(1);
        wobbleservo2.setPosition(0);
        sleep(500);
        /*
        wobble.setTargetPosition(10);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        wobble.setPower(Math.abs(.5));
        while (opModeIsActive() && (wobble.isBusy())) {


        }

        // Stop all motion;
        wobble.setPower(0);
*/

    }


}
