// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  Encoder armangle = new Encoder(0,1);
  double angle = 0;
  int lastGoal = 30;
  
  CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  WPI_TalonSRX m_arm1 = new WPI_TalonSRX(11);
  WPI_TalonSRX m_arm2 = new WPI_TalonSRX(12);
 
  CANSparkMax m_frontRight = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_shooter_bottom = new CANSparkMax(14, MotorType.kBrushless);
  CANSparkMax m_shooter_top = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(16, MotorType.kBrushless);

  DigitalInput limitSwitch = new DigitalInput(2);
  private final XboxController m_stick = new XboxController(0);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_rearRight, m_rearLeft);

  private static final double kP = 0.02;
  private static final double kI = 0.00001;
  private static final double kD = 0.000008;
  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  private static final double kPd = 0.005;
  private static final double kId = 0.0005;
  private static final double kDd = 0.00002;
  private final PIDController m_pidControllerDown = new PIDController(kPd, kId, kDd);

  private static final double kPb = 0.0016667;
  private static final double kIb = 0.0;
  private static final double kDb = 0.0;
  private final PIDController m_pidControllerBottom = new PIDController(kPb, kIb, kDb);

  private static final int targetID = 4;

  double m_set = 0;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rearRight.setInverted(true);
    m_frontRight.follow(m_rearRight);
    m_frontLeft.follow(m_rearLeft);
    armangle.reset();
    m_arm2.follow(m_arm1);
  }
  private void StartShoot() {
    m_shooter_bottom.set(0.7593);
    m_shooter_top.set(0.7593);
  }
  private void StopShoot() {
    m_shooter_bottom.set(0);
    m_shooter_top.set(0);
  }
  private void Intake() { intake.set(1); }
  private void Outake() { intake.set(-1); }
  private void Stoptake() { intake.set(0); }
  private void MoveArm() {
    if (m_stick.getAButtonPressed() && angle < 225) {
      angle = angle + 5;
    } else if(m_stick.getBButtonPressed()) {
      angle = angle - 5;
    }
  }
  private int setAngle(double distance){
    int angle = -1;
    if (distance > 0 && distance < 3) {
      angle = (int) ((-15.7143*Math.pow(distance, 2)) + (81.3095 * distance) + (36.4583));
    }
    return angle;
  }

  private void ArmSetAngle(int currentAngle, int goal) {
    if (goal != -1) {
      if (currentAngle <= 30 && currentAngle > goal) {
        m_set = -m_pidControllerBottom.calculate(currentAngle, goal);
      } else if (currentAngle > goal){
        m_set = -m_pidControllerDown.calculate(currentAngle, goal);
      } else {
        m_set = -m_pidController.calculate(currentAngle, goal);
      }

      lastGoal = goal;

      m_arm1.set(m_set);
    }
  }

  private void ArmSetAngleNoButton(int currentAngle, int goal) {
    if (goal != -1) {
      if (currentAngle <= 30 && currentAngle > goal) {
        m_set = -m_pidControllerBottom.calculate(currentAngle, goal);
      } else if (currentAngle > goal){
        m_set = -m_pidControllerDown.calculate(currentAngle, goal);
      } else {
        m_set = -m_pidController.calculate(currentAngle, goal);
      }

      m_arm1.set(m_set);
    }
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_stick.getLeftY(), -m_stick.getRightX());

    if (m_stick.getBackButton()) {
      boolean hasTargets = LimelightHelpers.getTV("");
      if (hasTargets && LimelightHelpers.getFiducialID("") == targetID) {
        
        Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
        
        double zDistance = target.getZ();
        System.out.println("Meters: " + zDistance);

        ArmSetAngle(armangle.get(), setAngle(zDistance));
      }
    } else { 
      ArmSetAngleNoButton(armangle.get(), lastGoal);
      System.out.println("Meters: N/A");
    }

    // MoveArm();

    // if (armangle.get() <= 30 && armangle.get() > angle) {
    //   m_set = -m_pidControllerBottom.calculate(armangle.get(), angle);
    // } else if (armangle.get() > angle){
    //   m_set = -m_pidControllerDown.calculate(armangle.get(), angle);
    // } else {
    //   m_set = -m_pidController.calculate(armangle.get(), angle);
    // }

    m_arm1.set(m_set);

    // Shooting
    if (m_stick.getXButton()) {
      StartShoot();
    } else {
      StopShoot();
    }

    // Intaking
    if (m_stick.getStartButton()) {
      Intake();
    } else if (m_stick.getYButton()){
      Outake();
    } else {
      Stoptake();
    }

    if (!limitSwitch.get()){
     armangle.reset();
    }

    if (angle > 225){
      angle = 225;
    } else if (angle < 0){
      angle = 0;
    }

    System.out.println("Encoder value: " + armangle.get());
    // System.out.println("Set Value: " + angle);
    // System.out.println(limitSwitch.get());
    // System.out.println("Motor Power: " + m_set);
    System.out.println("-----------------------------");
  }
}
