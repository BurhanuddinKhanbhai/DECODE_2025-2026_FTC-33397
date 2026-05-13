package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterModel;

@Autonomous(name = "RedAutoClose (Pedro)", group = "Pedro Pathing")
public class RedAutoClosePedro extends LinearOpMode {

    // =========================
    // RED POSES
    // =========================

    private static final Pose START = new Pose(-72, 72, Math.toRadians(180));

    private static final Pose FIRST_SHOT_POSE   = new Pose(-25, 20, Math.toRadians(130));
    private static final Pose BACK_SHOT_POSE    = new Pose(-28, 31, Math.toRadians(120));

    // If you want a different shoot spot for row 3 later, change this pose.
    private static final Pose THIRD_SHOT_POSE = new Pose(-28, 31, Math.toRadians(120));

    // LINEUP POSES
    private static final Pose FIRST_SHOT_LINEUP  = new Pose(5, 15, Math.toRadians(90));
    private static final Pose SECOND_SHOT_LINEUP = new Pose(40, 32, Math.toRadians(90));
    private static final Pose THIRD_SHOT_LINEUP  = new Pose(75, 32, Math.toRadians(90));

    private static final Pose INTAKE_ROW1_POSE  = new Pose(5, 95, Math.toRadians(90));
    private static final Pose ROW2_START_POSE   = new Pose(42, 32, Math.toRadians(90));
    private static final Pose INTAKE_ROW2_POSE  = new Pose(42, 110, Math.toRadians(90));

    private static final Pose ROW3_START_POSE   = new Pose(78, 32, Math.toRadians(90));
    private static final Pose INTAKE_ROW3_POSE  = new Pose(78, 115, Math.toRadians(90));

    private static final Pose FINAL_SHOT_POSE   = new Pose(-27, 37, Math.toRadians(120));

    // =========================
    // Subsystems
    // =========================
    private Follower follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private IntakeFeederSubsystem intakeFeeder;
    private LogitechCameraSubsystem logitech;

    // =========================
    // Tuning
    // =========================
    private static final double RPM_TOL = 75;
    private static final long SPINUP_TIMEOUT_MS = 1800;
    private static final long FEED_MS = 850;
    private static final double INTAKE_FEED_PWR = 1.0;

    @Override
    public void runOpMode() {

        // ---------- Hardware ----------
        DcMotorEx leftFly = hardwareMap.get(DcMotorEx.class, "flywheel2");
        DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class, "flywheel");

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx feeder = hardwareMap.get(DcMotorEx.class, "intakeMotor2");

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");

        WebcamName logitechCamera = hardwareMap.get(WebcamName.class, "logitech");

        shooter = new ShooterSubsystem(leftFly, rightFly);
        hood = new HoodSubsystem(hoodServo);
        intakeFeeder = new IntakeFeederSubsystem(intake, feeder);
        logitech = new LogitechCameraSubsystem(logitechCamera);
        logitech.init();

        // ---------- Pedro ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        // =========================
        // Build Paths
        // =========================

        PathChain toFirstShot      = line(START, FIRST_SHOT_POSE);
        PathChain toBackShot       = line(FIRST_SHOT_POSE, BACK_SHOT_POSE);

        // ROW 1
        PathChain toFirstLineup    = line(BACK_SHOT_POSE, FIRST_SHOT_LINEUP);
        PathChain lineupToRow1     = line(FIRST_SHOT_LINEUP, INTAKE_ROW1_POSE);
        PathChain backToShoot1     = line(INTAKE_ROW1_POSE, BACK_SHOT_POSE);

        // ROW 2
        PathChain toRow2Start      = line(BACK_SHOT_POSE, ROW2_START_POSE);
        PathChain toSecondLineup   = line(ROW2_START_POSE, SECOND_SHOT_LINEUP);
        PathChain lineupToRow2     = line(SECOND_SHOT_LINEUP, INTAKE_ROW2_POSE);

        // NEW: back after ROW 2 to shoot again
        PathChain backToShoot2     = line(INTAKE_ROW2_POSE, BACK_SHOT_POSE);

        // ROW 3 (using your new poses)
        PathChain toRow3Start      = line(THIRD_SHOT_POSE, ROW3_START_POSE);     // from shoot spot -> lane start
        PathChain toThirdLineup    = line(ROW3_START_POSE, THIRD_SHOT_LINEUP);   // lane start -> lineup at x=75
        PathChain lineupToRow3     = line(THIRD_SHOT_LINEUP, INTAKE_ROW3_POSE);  // lineup -> intake row 3

        // Final
        PathChain toFinalShot      = line(INTAKE_ROW3_POSE, FINAL_SHOT_POSE);

        telemetry.addLine("RedAutoCloseReady");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // AUTO SEQUENCE
        // =========================

        followAndWait(toFirstShot);
        shootBurst3();

        followAndWait(toBackShot);

        // ROW 1
        followAndWait(toFirstLineup);
        setIntake(true);
        followAndWait(lineupToRow1);
        setIntake(false);

        followAndWait(backToShoot1);
        shootBurst3();

        // ROW 2
        followAndWait(toRow2Start);
        followAndWait(toSecondLineup);
        setIntake(true);
        followAndWait(lineupToRow2);
        setIntake(false);

        // NEW: shoot after row 2
        followAndWait(backToShoot2);
        shootBurst3();

        // ROW 3
        followAndWait(toRow3Start);
        followAndWait(toThirdLineup);
        setIntake(true);
        followAndWait(lineupToRow3);
        setIntake(false);

        // Final Shot after row 3
        followAndWait(toFinalShot);
        shootBurst3();

        safeStopAll();
    }

    // =========================
    // Path Helper
    // =========================
    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private void followAndWait(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            logitech.update();
            shooter.update();
            hood.update();
            intakeFeeder.update();
            idle();
        }
    }

    // =========================
    // Shooting
    // =========================
    private void shootBurst3() {

        shooter.setEnabled(true);

        ElapsedTime aimTimer = new ElapsedTime();
        aimTimer.reset();
        while (opModeIsActive() && aimTimer.milliseconds() < 300) {
            logitech.update();
            shooter.update();
            hood.update();
            telemetry.addLine("Aiming...");
            telemetry.update();
            idle();
        }

        double dist = logitech.getDistanceInches();
        double targetRPM = ShooterModel.rpmFromDistance(dist);
        double hoodPos = ShooterModel.hoodFromDistance(dist);

        shooter.setTargetRPM(targetRPM);
        hood.setTargetPos(hoodPos);

        ElapsedTime spin = new ElapsedTime();
        spin.reset();

        while (opModeIsActive() && spin.milliseconds() < SPINUP_TIMEOUT_MS) {

            logitech.update();
            shooter.update();
            hood.update();

            double currentRPM = shooter.getRPM();
            boolean ready = shooter.atSpeed(RPM_TOL);

            telemetry.addLine("=== SHOOTER DATA ===");
            telemetry.addData("Distance (in)", "%.1f", dist);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("At Speed?", ready);
            telemetry.update();

            if (ready) break;
            idle();
        }

        ElapsedTime feed = new ElapsedTime();
        feed.reset();

        while (opModeIsActive() && feed.milliseconds() < FEED_MS) {

            intakeFeeder.setIntakePower(INTAKE_FEED_PWR);
            intakeFeeder.setFeederPower(INTAKE_FEED_PWR);

            shooter.update();
            intakeFeeder.update();

            telemetry.addLine("=== FEEDING ==");
            telemetry.addData("Distance (in)", "%.1f", dist);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", shooter.getRPM());
            telemetry.update();

            idle();
        }

        intakeFeeder.setIntakePower(0);
        intakeFeeder.setFeederPower(0);
        intakeFeeder.update();

        sleep(100);
    }

    private void setIntake(boolean on) {
        if (on) {
            intakeFeeder.setIntakePower(1.0);
        } else {
            intakeFeeder.setIntakePower(0.0);
        }
        intakeFeeder.setFeederPower(0.0);
        intakeFeeder.update();
    }

    private void safeStopAll() {
        shooter.setEnabled(false);
        intakeFeeder.setIntakePower(0);
        intakeFeeder.setFeederPower(0);
        intakeFeeder.update();
        if (logitech != null) {
            logitech.close();
        }
    }
}
