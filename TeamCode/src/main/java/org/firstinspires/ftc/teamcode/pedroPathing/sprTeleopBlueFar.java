package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.function.Supplier;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Configurable
@TeleOp
public class sprTeleopBlueFar extends OpMode {
    private double maxVelocityOuttake = 2000;
    private double F = 32767.0/maxVelocityOuttake;
    private double kP = F * .1;
    private double kI = kP * .1;
    private double kD = 0;
    private double pos = 5.0;

    private Follower follower;
    private int[] c1Def = {713, 1311, 1164};
    private int[] c2Def = {392, 895, 767};
    private int[] c3Def = {461, 895, 796};
    private static final int MIN_TOTAL = 1700; // increase for bright floor

    private static final int BALL_PRESENT_MIN = 3000;  // higher because sideways
    private static final double GREEN_DOMINANCE = 0.45;
    private static final double PURPLE_RB_MIN = 0.68;

    private Limelight3A limelight;
    private ColorSensor c1, c2, c3;
    private Servo fanRotate, cam, park1, park2, arm1, arm2,arm3, shooterAngle;
    private DcMotorEx outtake1, backspinRoller, outtake2, turret;
    private DcMotorSimple intake, rightFront, leftFront, rightRear, leftRear;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive, isCam;
    private Supplier<PathChain> pathChain1, pathChain2;
    private TelemetryManager telemetryM;
    private Integer lockedTagId = null;
    private ElapsedTime tagLostTimer = new ElapsedTime();
    private static final double TAG_LOST_TIMEOUT = 0.4; // seconds
    private double turretKp = 0.015;      // proportional gain
    private double turretKd = 0.008;      // derivative gain
    private double countsPerDegree = 5.56; // adjust for your motor/gearbox
    private double turretMin = 0;      // min encoder count
    private double turretMax = 2500;   // max encoder count
    private double turretMaxVel = 500;    // max velocity (encoder counts per sec)
    private double lastTurretError = 0;   // for derivative calculation
    private double turretTarget = 0;
    ElapsedTime artifactTimer = new ElapsedTime();
    boolean artifactRunning = false;
    int artifactState = 0;
    private static final double TICKS_PER_REV = 537.7; // goBILDA 5202/5203
    private static final double GEAR_RATIO = 1.0;      // change if geared
    private static final double TICKS_PER_DEGREE =
            (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // Turret limits (ENCODER TICKS)
    private static final int TURRET_MIN = 0;
    private static final int TURRET_MAX = 1000;

    private double lastTurretVelocity = 0;

    // Tracking
    private double turretTargetTicks = 0;
    private double lastError = 0;
    private boolean slowMode = false;

    private double currPosFan = .05, camPos = 1, currRelease=-.01;
    private double fanPos1 = .1, fanPos2 =  .145, fanPos3 = .195, fanPos4 = .24;
    private double upPos1 = .075, upPos2 = .125, upPos3 =.17;
    private boolean x = true;

    private boolean x2 = true;
    private int count = 1, count3 = 1, targetVel = 1150, rollerVel = 1250;
    private int count2 = 1;
    private int motorVel = 0;
    private int angle = 0;
    private double fastModeMultiplier = .3;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(10,10, Math.toRadians(-90)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        c1 = hardwareMap.get(ColorSensor.class, "c1");
        c2 = hardwareMap.get(ColorSensor.class, "c2");
        c3 = hardwareMap.get(ColorSensor.class, "c3");
        leftFront = hardwareMap.get(DcMotorSimple.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorSimple.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorSimple.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorSimple.class, "rightFront");
        intake = hardwareMap.get(DcMotorSimple.class,  "intake");
        pathChain1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(59, 18))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(294), 0.8))
                .build();
        pathChain2 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(53, 96))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(320), 0.8))
                .build();
        c1.enableLed(true);
        c2.enableLed(true);
        c3.enableLed(true);
        limelight.pipelineSwitch(6);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidf = new PIDFCoefficients(5, 0, 0, 0); // kP=5, kI=0, kD=0, kF=0 (tune these)
        turret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        turret.setVelocity(0); // start stopped
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setVelocityPIDFCoefficients(kP,kI,kD,F);
        outtake2.setVelocityPIDFCoefficients(kP,kI,kD,F);

    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(0);
        turret.setTargetPosition(0);
        shooterAngle.setPosition(0);
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        //updateTurretTracking();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * fastModeMultiplier,
                    -gamepad1.left_stick_x * fastModeMultiplier,
                    -gamepad1.right_stick_x * fastModeMultiplier,
                    true // Robot Centric
            );
            if (gamepad1.rightStickButtonWasPressed()) {
                fastModeMultiplier = 0.75;
            }
            if (gamepad1.rightStickButtonWasReleased()) {
                fastModeMultiplier = .3;
            }
            if(gamepad1.aWasPressed()){
                arm1.setPosition(1);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm1.setPosition(0);
            }
            if(gamepad1.bWasPressed()){
                arm2.setPosition(1);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm2.setPosition(0);
            }
            if(gamepad1.xWasPressed()){
                arm3.setPosition(1);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm3.setPosition(0);
            }
            if(gamepad1.left_trigger > 0){
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(1);
            }
            else if(gamepad1.left_trigger <= 0){
                intake.setPower(0);
            }
            if(gamepad1.right_trigger > 0){
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            }
            if(gamepad1.dpadRightWasPressed()){
                motorVel = 800;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
            }
            if(gamepad1.dpadUpWasPressed()){
                motorVel += 50;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
            }
            if(gamepad1.dpadUpWasPressed()){
                motorVel -= 50;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
            }
            if(gamepad2.dpadUpWasPressed()){
                shooterAngle.setPosition(1);
            }
            if(gamepad2.dpadDownWasPressed()){
                shooterAngle.setPosition(.5);
            }
            if(gamepad1.right_trigger <= 0){
                intake.setPower(0);
            }
            if(gamepad1.yWasPressed()){
                arm1.setPosition(1);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm1.setPosition(0);
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm2.setPosition(1);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm2.setPosition(0);
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm3.setPosition(1);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                arm3.setPosition(0);

            }
        }
//        String c1Color = detectColor(c1);
//        String c2Color = detectColor(c2);
//        String c3Color = detectColor(c3);


        String c1Color = detectColor1(c1);
        String c2Color = detectColor2(c2);
        String c3Color = detectColor3(c3);
        float[] hsv = new float[3];
        Color.RGBToHSV(c1.red(), c1.green(), c1.blue(), hsv);
        telemetry.addData("C1 HSV", "H: %.1f S: %.2f V: %.2f", hsv[0], hsv[1], hsv[2]);

        Color.RGBToHSV(c2.red(), c2.green(), c2.blue(), hsv);
        telemetry.addData("C2 HSV", "H: %.1f S: %.2f V: %.2f", hsv[0], hsv[1], hsv[2]);

        Color.RGBToHSV(c3.red(), c3.green(), c3.blue(), hsv);
        telemetry.addData("C3 HSV", "H: %.1f S: %.2f V: %.2f", hsv[0], hsv[1], hsv[2]);
        telemetry.addData("C1 Color", c1Color);
        telemetry.addData("C2 Color", c2Color);
        telemetry.addData("C3 Color", c3Color);
        telemetry.update();


    }
    private String detectColor1(ColorSensor c) {
        // Check proximity first (distance to object)
        if(c instanceof DistanceSensor) {
            double distance = ((DistanceSensor)c).getDistance(DistanceUnit.MM);
            if(distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if(hue > 160 && hue < 170 && sat > 0.45 && val > 5) return "GREEN";

        // Purple range (tweak if needed)
        if(hue > 170 && sat < .5 && val < 5.6) return "PURPLE";

        return "UNKNOWN";
    }
    private String detectColor2(ColorSensor c) {
        // Check proximity first (distance to object)
        if(c instanceof DistanceSensor) {
            double distance = ((DistanceSensor)c).getDistance(DistanceUnit.MM);
            if(distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if(hue > 160 && hue < 170 && sat > .57 && val > 3.5) return "GREEN";

        // Purple range (tweak if needed)
        if(hue > 160 && hue < 170 && sat < .57 && val > 3.5) return "PURPLE";

        return "UNKNOWN";
    }
    private String detectColor3(ColorSensor c) {
        // Check proximity first (distance to object)
        if(c instanceof DistanceSensor) {
            double distance = ((DistanceSensor)c).getDistance(DistanceUnit.MM);
            if(distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if(hue > 160 && hue < 170 && sat > 0.45 && val > 3.5) return "GREEN";

        // Purple range (tweak if needed)
        if(hue > 170 && sat < .5 && val < 5) return "PURPLE";

        return "UNKNOWN";
    }
    private void updateTurretTracking() {
        // Make sure Limelight is using pipeline 6
        limelight.pipelineSwitch(6);

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turret.setPower(0);
            telemetry.addData("Turret", "No target");
            return;
        }

        // Grab fiducials (AprilTags)
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.size() == 0) {
            turret.setPower(0);
            telemetry.addData("Turret", "No tag detected");
            return;
        }

        // Just pick the first detected tag
        LLResultTypes.FiducialResult tag = tags.get(0);
        int tagId = tag.getFiducialId(); // <-- add this

        // Horizontal offset to target
        double limelightYaw = tag.getTargetXDegrees(); // degrees from center

        // Adjust for robot heading
        double robotHeading = follower.getHeading(); // implement using IMU
        turretTarget = robotHeading + limelightYaw;

        // Convert turret encoder counts to degrees
        double currentTurretAngle = turret.getCurrentPosition() / 4.0; // adjust for counts per degree

        // PID calculation
        double error = turretTarget - currentTurretAngle;

        // Wrap error to [-180,180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double derivative = error - lastTurretError;
        double turretPower = turretKp * error + turretKd * derivative;

        // Clamp power and respect physical limits
        if (currentTurretAngle < turretMin && turretPower < 0) turretPower = 0;
        if (currentTurretAngle > turretMax && turretPower > 0) turretPower = 0;
        turretPower = Math.max(-1, Math.min(1, turretPower));

        turret.setPower(turretPower);
        lastTurretError = error;

        // Telemetry
        telemetry.addData("Turret Target", turretTarget);
        telemetry.addData("Turret Angle", currentTurretAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Tag ID", tagId); // <-- shows which AprilTag is being tracked
    }






}