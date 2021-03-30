package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AuxHardwareMap {
    /* Public OpMode members. */

    public DcMotor shooter = null;
    public DcMotor belt = null;
    public DcMotor wobble = null;
    public Servo shooter_Servo = null;
    public Servo wobbleservo1 = null;
    public Servo wobbleservo2 = null;
    public ElapsedTime runtime = new ElapsedTime();


    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public AuxHardwareMap() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        shooter = hwMap.get(DcMotor.class, "flywheel");
        belt = hwMap.get(DcMotor.class, "belt");
        wobble = hwMap.get(DcMotor.class, "wobble");
        wobble = hwMap.get(DcMotor.class, "wobble");
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.
        shooter_Servo = hwMap.get(Servo.class, "shooter");
        wobbleservo1 = hwMap.get(Servo.class, "wobble1");
        wobbleservo2 = hwMap.get(Servo.class, "wobble2");


        //Move Servos to Init Position
        wobbleservo1.setPosition(.5);
        wobbleservo2.setPosition(.5);

    }
}
