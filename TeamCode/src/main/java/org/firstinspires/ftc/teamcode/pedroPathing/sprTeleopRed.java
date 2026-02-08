package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class sprTeleopRed extends OpMode {
    // Field coordinates for your goal tag center (in same units as Pedro follower)
    private static final double TAG_X = 6.0;
    private static final double TAG_Y = 144.0;

    // Turret encoder limits you measured
    private static final int TURRET_MIN_TICKS = -1000;
    private static final int TURRET_MAX_TICKS =  1000;

    // Motor/gear
    private static final double TICKS_PER_REV = 537.7;
    private static final double GEAR_RATIO    = 1.0; // change if you have gearing
    private double lastTurretErrorTicks = 0;

    private double searchDirection = 1.0; // +1 = right, -1 = left
    private static final double SEARCH_POWER = .5;
    boolean turretLocked = false;


    boolean centering = false;
    private static final double TARGET_X = 2;
    private static final double TARGET_Y = 135;
    private double lastErrorDeg = 0;// start big so you SEE motion
    private static final int RIGHT_LIMIT = 800;  // ticks – set from your tests
    private static final double KP_TRACK = 0.04;
    private static final double MAX_TRACK_POWER = 0.5;
    private static final double CENTER_DEADBAND_DEG = 1.0;
    private boolean scanningRight = true;
    // use your real value here:
    private static final double TURRET_KP = 0.015;  // from 0.04 → HALF
    private static final double TURRET_KD = 0.002;  // from 0.004 → HALF


    private static final double TURRET_MAX_POWER = 0.25;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // limits in TICKS, not "random 0–250"
    // Turret encoder limits
    private double lastTurretError = 0;
    private static final double KP = 0.012;
    private static final double KD = 0.008;

    private static final int TURRET_MIN = -1000;
    private static final int TURRET_MAX = 1000;

    // Small movement step while searching
    static int SEARCH_STEP = 10;


    static final double TX_CENTER_THRESHOLD = 1.5;
    static final int AIM_STEP = 5;       // slower when detected


    private double maxVelocityOuttake = 2400;
    private double F = 32767.0 / maxVelocityOuttake;
    private double kP = F * .1;
    private double kI = 0;
    private double kD = 0;

    private double Ftur = 32767.0 / maxVelocityOuttake;
    private double kPtur = F * .1;
    private double kItur = kP * .1;
    private double kDtur = 0;
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
    private Servo fanRotate, cam, park1, park2, arm1, arm2, arm3, shooterAngle;
    private DcMotorEx outtake1, backspinRoller, outtake2, turret;
    private DcMotorSimple intake, rightFront, leftFront, rightRear, leftRear;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive, isCam;
    private Supplier<PathChain> pathChain1, pathChain2;
    private TelemetryManager telemetryM;
    private Integer lockedTagId = null;
    private static final double VISION_BLEND = 0.05; // small weight for tx correction
    private ElapsedTime tagLostTimer = new ElapsedTime();
    private static final double TAG_LOST_TIMEOUT = 0.4; // seconds     // proportional gain
    private double turretKd = 0.008;      // derivative gain
    private double countsPerDegree = 5.56; // adjust for your motor/gearbox
    private double turretMin = 0;      // min encoder count
    private double turretMax = 250;
    // scanning power when no tag// max encoder count
    private double turretMaxVel = 500;    // max velocity (encoder counts per sec)
    private double turretTarget = 0;

    private double turretKp = 0.02;       // smaller gain for smooth centering
    double lastTx = 0;
    ElapsedTime artifactTimer = new ElapsedTime();
    boolean artifactRunning = false;
    int artifactState = 0;


    // Turret limits (ENCODER TICKS)

    private double lastTurretVelocity = 0;

    // Tracking
    private double turretTargetTicks = 0;
    private double lastError = 0;
    private boolean slowMode = false;
    private int turretPos = 0;
    private ElapsedTime searchTimer = new ElapsedTime();
    private boolean searching = false;
    private double searchPower = 0.15;

    private double currPosFan = .05, camPos = 1, currRelease = -.01;
    private double fanPos1 = .1, fanPos2 = .145, fanPos3 = .195, fanPos4 = .24;
    private double upPos1 = .075, upPos2 = .125, upPos3 = .17;
    private boolean x = true;

    private boolean x2 = true;
    private int count = 1, count3 = 1, targetVel = 1150, rollerVel = 1250;
    private int count2 = 1;
    private int motorVel = 0;
    private int angle = 0;
    private double fastModeMultiplier = .3;

    // Turret constants
    private static final double TURRET_TICKS_PER_DEGREE = 5.56; // your motor/gear setup


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 6, Math.toRadians(-90)));
        follower.update();
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
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
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
        limelight.pipelineSwitch(7);
        turret.setDirection(DcMotorSimple.Direction.FORWARD); // or REVERSE if needed
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.setPollRateHz(30); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setVelocityPIDFCoefficients(kP, kI, kD, F);

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
        turret.setPower(0); // start stopped
        shooterAngle.setPosition(1);
        searchTimer = new ElapsedTime();

    }

    @Override
    public void loop() {
        //Call this once per loop
        updateTurretTracking();
        updateArmShooter();
        follower.update();
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
            fastModeMultiplier = .5;
        }
        if (gamepad1.rightStickButtonWasReleased()) {
            fastModeMultiplier = .9;
        }
        if (gamepad1.aWasPressed()) {
            fireArmNonBlocking(arm1);
        }
            if (gamepad1.bWasPressed()) {
                fireArmNonBlocking(arm2);
            }
            if (gamepad1.xWasPressed()) {
                fireArmNonBlocking(arm3);
            }
            if (gamepad1.left_trigger > 0) {
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(1);
            } else if (gamepad1.left_trigger <= 0) {
                intake.setPower(0);
            }
            if (gamepad1.right_trigger > 0) {
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(1);
            }
            if (gamepad1.dpadRightWasPressed()) {
                motorVel = 1120;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
                shooterAngle.setPosition(.5);
            }
            if(gamepad1.dpadLeftWasPressed()){
                motorVel = 1350;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
                shooterAngle.setPosition(.4);
            }
            if (gamepad1.leftStickButtonWasPressed()) {
                outtake1.setPower(0);
                outtake2.setPower(0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                motorVel += 50;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
            }
            if (gamepad1.dpadDownWasPressed()) {
                motorVel -= 50;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
            }
            if (gamepad2.dpadUpWasPressed()) {
                shooterAngle.setPosition(.7);
            }
            if (gamepad2.dpadDownWasPressed()) {
                shooterAngle.setPosition(1);
            }
            if (gamepad1.right_trigger <= 0 && gamepad1.left_trigger <= 0) {
                intake.setPower(0);
            }
            if (gamepad1.yWasPressed()) {
                shootInOrder(22);
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
        telemetry.addData("Outtake 1: ", outtake1.getVelocity());
        telemetry.addData("Outtake 2", outtake2.getVelocity());
        telemetry.update();


    }
    enum ArmState {
        IDLE,
        EXTEND,
        RETRACT
    }

    ArmState armState = ArmState.IDLE;
    long armTimer = 0;
    Servo activeArm = null;

    public void updateArmShooter() {
        long now = System.currentTimeMillis();

        switch (armState) {
            case IDLE:
                break;

            case EXTEND:
                if (now - armTimer > 200) {
                    activeArm.setPosition(0);
                    armState = ArmState.RETRACT;
                    armTimer = now;
                }
                break;

            case RETRACT:
                if (now - armTimer > 250) {
                    armState = ArmState.IDLE;
                    activeArm = null;
                }
                break;
        }
    }

    private String detectColor1(ColorSensor c) {
        // Check proximity first (distance to object)
        if (c instanceof DistanceSensor) {
            double distance = ((DistanceSensor) c).getDistance(DistanceUnit.MM);
            if (distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if (hue > 150 && hue < 170 && sat > 0.5) return "GREEN";

        // Purple range (tweak if needed)
        if (hue > 180 && sat < .45) return "PURPLE";

        return "UNKNOWN";
    }

    private String detectColor2(ColorSensor c) {
        // Check proximity first (distance to object)
        if (c instanceof DistanceSensor) {
            double distance = ((DistanceSensor) c).getDistance(DistanceUnit.MM);
            if (distance > 70) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if (hue > 160 && hue < 170 && sat > .6) return "GREEN";

        // Purple range (tweak if needed)
        if (hue > 180 && sat < .53) return "PURPLE";

        return "UNKNOWN";
    }

    private String detectColor3(ColorSensor c) {
        // Check proximity first (distance to object)
        if (c instanceof DistanceSensor) {
            double distance = ((DistanceSensor) c).getDistance(DistanceUnit.MM);
            if (distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if (hue > 150 && hue < 160 && sat > 0.6) return "GREEN";

        // Purple range (tweak if needed)
        if (hue > 200 && sat < .5) return "PURPLE";

        return "UNKNOWN";
    }




    // --- Smoothed Turret Tracking ---


    private static final double TURRET_MIN_POWER = .7; // small power so search moves slowly

    private static final double TX_DEADZONE = 1.5; // degrees
    private static final double MAX_TURRET_VEL = 800; // ticks/sec max
    private static final double MIN_TURRET_VEL = 100; // slow near center

    private static final double CENTER_ENTER = 2.0; // deg: lock when inside
    private static final double CENTER_EXIT  = 3.0; // deg: unlock when outside
    private static final double TRACK_KP     = 0.01;
    private static final double MAX_POWER    = .4;
    private boolean turretCentered = false;
    private static final double CENTER_OFFSET_DEG = 0; // aim left of tag

    private void updateTurretTracking() {
        limelight.pipelineSwitch(8);

        LLResult result = limelight.getLatestResult();
        boolean tagSeen = false;
        double tx = 0;

        if (result != null && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty()) {

            tx = result.getFiducialResults().get(0).getTargetXDegrees();
            tagSeen = true;
        }

        int pos = turret.getCurrentPosition();
        boolean atLeftLimit  = pos <= -995;
        boolean atRightLimit = pos >=  995;

        double power = 0;

        if (tagSeen) {
            // ---------------- VAW VISION TRACK ----------------
            double errorDeg = -(tx - CENTER_OFFSET_DEG);

            // HYSTERESIS LOCK
            if (!turretCentered && Math.abs(errorDeg) < CENTER_ENTER) {
                turretCentered = true;
            }
            if (turretCentered && Math.abs(errorDeg) > CENTER_EXIT) {
                turretCentered = false;
            }

            if (!turretCentered) {
                power = TRACK_KP * errorDeg;
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            } else {
                power = 0;
            }

            // Hard stop at limits
            if ((atLeftLimit && power < 0) || (atRightLimit && power > 0)) {
                power = 0;
            }

        } else {
            // ---------------- FAST SEARCH ----------------
            turretCentered = false;

            power = searchDirection * .5;

            if (atLeftLimit)  searchDirection = 1;
            if (atRightLimit) searchDirection = -1;
        }

        turret.setPower(power);

        telemetry.addData("TX", String.format("%.2f", tx));
        telemetry.addData("ErrDeg", String.format("%.2f", -(tx - CENTER_OFFSET_DEG)));
        telemetry.addData("Centered", turretCentered);
        telemetry.addData("Power", String.format("%.2f", power));
    }

    public void fireArmNonBlocking(Servo arm) {
        if (armState == ArmState.IDLE) {
            activeArm = arm;
            arm.setPosition(1);
            armTimer = System.currentTimeMillis();
            armState = ArmState.EXTEND;
        }
    }


    private String[] getDesiredPattern(int num) {
        switch (num) {
            case 21: return new String[]{"GREEN", "PURPLE", "PURPLE"}; // GPP
            case 22: return new String[]{"PURPLE", "GREEN", "PURPLE"}; // PGP
            case 23: return new String[]{"PURPLE", "PURPLE", "GREEN"}; // PPG
            default: return null;
        }
    }
    public void shootInOrder(int num) {

        String col1 = detectColor1(c1);
        String col2 = detectColor2(c2);
        String col3 = detectColor3(c3);

        // Arrays to keep things clean
        String[] colors = {col1, col2, col3};
        Servo[] arms   = {arm1, arm2, arm3};

        String[] pattern;

        if (num == 21)      pattern = new String[]{"GREEN", "PURPLE", "PURPLE"};
        else if (num == 22) pattern = new String[]{"PURPLE", "GREEN", "PURPLE"};
        else if (num == 23) pattern = new String[]{"PURPLE", "PURPLE", "GREEN"};
        else return; // invalid number

        boolean[] used = {false, false, false};

        // Shoot in pattern order
        for (String targetColor : pattern) {
            for (int i = 0; i < 3; i++) {
                if (!used[i] && colors[i].equals(targetColor)) {
                    fireArmNonBlocking(arms[i]);
                    used[i] = true;
                    break;
                }
            }
        }
    }












}