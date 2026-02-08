package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;
import com.qualcomm.hardware.limelightvision.LLResult;

@Configurable
@TeleOp
public class sprTeleopBlue extends OpMode {
    // Field coordinates for your goal tag center (in same units as Pedro follower)
    private ElapsedTime runTime = new ElapsedTime();
    private double searchDirection = 1.0; // +1 = right, -1 = left
    private RevBlinkinLedDriver ledLights;
    private double maxVelocityOuttake = 2400;
    private double F = 32767.0 / maxVelocityOuttake;
    private double kP = F * 1.2;
    private double kI = 0;
    private double kD = 0;
    private Follower follower;

    private Limelight3A limelight;
    private ColorSensor c1, c2, c3;
    private Servo fanRotate, cam, park1, park2, arm1, arm2, arm3, shooterAngle, lift1, lift2;
    private DcMotorEx outtake1, backspinRoller, outtake2, turret;
    private DcMotorSimple intake, rightFront, leftFront, rightRear, leftRear;
    private Supplier<PathChain> pathChain1, pathChain2;
    private ElapsedTime tagLostTimer = new ElapsedTime();

    ElapsedTime artifactTimer = new ElapsedTime();
    boolean artifactRunning = false;
    int artifactState = 0;



    private ElapsedTime searchTimer = new ElapsedTime();

    private int motorVel = 0;
    private double fastModeMultiplier = .3;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 6, Math.toRadians(-90)));
        follower.update();
        ledLights = hardwareMap.get(RevBlinkinLedDriver.class, "ledLights");
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
        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
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
        lift1.setPosition(0);
        lift2.setPosition(0);
        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(0);
        turret.setPower(0); // start stopped
        shooterAngle.setPosition(1);
        runTime = new ElapsedTime();
        runTime.startTime();
        searchTimer = new ElapsedTime();

    }

    @Override
    public void loop() {
        //Call this once per loop
        led();
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
            fastModeMultiplier = 1;
        }
        if (gamepad1.rightStickButtonWasReleased()) {
            fastModeMultiplier = .5;
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
                motorVel = 1180;
                outtake1.setVelocity(motorVel);
                outtake2.setVelocity(motorVel);
                shooterAngle.setPosition(1);

            }
            if(gamepad1.dpadLeftWasPressed()){
                motorVel = 1300;
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
            if(gamepad2.rightBumperWasPressed()){
                    lift1.setPosition(1);
                    lift2.setPosition(1);
                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    while(true){
                        leftFront.setPower(.5);
                        rightFront.setPower(.5);
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
        telemetry.addData("Outtake 1: ", outtake1.getVelocity());
        telemetry.addData("Outtake 2", outtake2.getVelocity());
        telemetry.addData("Pos", turret.getCurrentPosition());
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
        limelight.pipelineSwitch(7);

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
        boolean atLeftLimit  = pos <= -550;
        boolean atRightLimit = pos >=  600;

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
    public void led(){
        if(!detectColor1(c1).equals("NONE") && !detectColor2(c2).equals("NONE") && !detectColor3(c3).equals("NONE")){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        }
        else{
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        double seconds = runTime.seconds();
        if(seconds >= 110){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        }
        else if(seconds >= 100){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
        }
    }












}