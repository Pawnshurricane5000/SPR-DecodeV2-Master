# Spare Parts Robotics #11848 — DECODE (2025–26)

Competition code for FTC Team 11848's 2025–26 DECODE season. Written in Java using Android Studio with the FTC SDK v11.0.

## About

Spare Parts Robotics is a community-based FTC team from Lake Villa, Illinois. We've competed for 6 seasons, building 4+ robots, writing 50+ autonomous programs, and accumulating 10,000+ outreach hours impacting 100,000+ people.

## Code

Custom team code: `TeamCode/src/main/java/.../teamcode/`

This repo builds on our [Into The Deep codebase](https://github.com/Pawnshurricane5000/SPR-IntoTheDeep-master-master), carrying forward our odometry, PID control, and hardware abstraction systems into the new season.

## Technical Stack

- **Odometry** — 3-wheel dead-wheel localization via Roadrunner
- **PID Control** — closed-loop motor positioning for drivetrain, slides, and arm
- **Sensor Fusion** — encoder feedback, touch sensor resets, distance sensors
- **Computer Vision** — OpenCV color processing, AprilTag detection
- **Mecanum Drivetrain** — field-centric drive with dual gamepad control

## Previous Season Results

- FTC IL State Championship — Inspire Award (3rd, 2025)
- Regional Inspire Award (2nd, 2025)
- Regional Design Award (3rd, 2024)

## Built With

Java · Android Studio · FTC SDK 11.0 · Roadrunner

## Author

**Hemanth Samayamantri** — Programming Lead · [@Pawnshurricane5000](https://github.com/Pawnshurricane5000)
