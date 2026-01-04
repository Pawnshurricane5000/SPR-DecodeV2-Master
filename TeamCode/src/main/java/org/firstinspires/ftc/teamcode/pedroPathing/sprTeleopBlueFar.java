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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class sprTeleopBlueFar extends OpMode {
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
    private Servo fanRotate, cam, park1, park2, arm1, arm2,arm3;
    private DcMotorEx outtake1, backspinRoller, outtake2;
    private DcMotorSimple intake, rightFront, leftFront, rightRear, leftRear;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive, isCam;
    private Supplier<PathChain> pathChain1, pathChain2;
    private TelemetryManager telemetryM;

    ElapsedTime artifactTimer = new ElapsedTime();
    boolean artifactRunning = false;
    int artifactState = 0;

    private boolean slowMode = false;

    private double currPosFan = .05, camPos = 1, currRelease=-.01;
    private double fanPos1 = .1, fanPos2 =  .145, fanPos3 = .195, fanPos4 = .24;
    private double upPos1 = .075, upPos2 = .125, upPos3 =.17;
    private boolean x = true;

    private boolean x2 = true;
    private int count = 1, count3 = 1, targetVel = 1150, rollerVel = 1250;
    private int count2 = 1;
    private double motorPower1 = .63;
    private double fastModeMultiplier = .3;

    @Override
    public void init() {
        BlueAuto1 x = new BlueAuto1();
        follower = Constants.createFollower(hardwareMap);
        MecanumConstants drive = new MecanumConstants();
        follower.setStartingPose(x.getFinalPose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");
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

    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

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
        telemetry.addData("C1 RGB", "R:%d G:%d B:%d", c1.red(), c1.green(), c1.blue());
        telemetry.addData("C2 RGB", "R:%d G:%d B:%d", c2.red(), c2.green(), c2.blue());
        telemetry.addData("C3 RGB", "R:%d G:%d B:%d", c3.red(), c3.green(), c3.blue());
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




}