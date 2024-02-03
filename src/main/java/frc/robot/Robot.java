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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  Encoder armangle = new Encoder(0,1);
  double angle = 0;
  
  CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  //MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft); no good
  WPI_TalonSRX m_arm1 = new WPI_TalonSRX(11);
  WPI_TalonSRX m_arm2 = new WPI_TalonSRX(12);
 
  //CANSparkMax m_frontRight = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_frontRight = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_rearRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_shooter_bottom = new CANSparkMax(14, MotorType.kBrushless);
  CANSparkMax m_shooter_top = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(16, MotorType.kBrushless);

  DigitalInput limitSwitch = new DigitalInput(2);
  //MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight); no good
  private final XboxController m_stick = new XboxController(0);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_rearRight, m_rearLeft);
  private static final double kP = 0.02;
  private static final double kI = 0.0005;
  private static final double kD = 0.0005;
  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  private static final double kPd = 0.005;
  private static final double kId = 0.0;
  private static final double kDd = 0.001;
  private final PIDController m_pidControllerDown = new PIDController(kPd, kId, kDd);
  int countmax = 213;
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
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_stick.getLeftY(), -m_stick.getRightX());
    if (m_stick.getAButtonPressed() && armangle.get()<=120) {
        //m_arm1.set(m_pidController.calculate(armangle.get(), ));
        angle = angle + 10;
    }
    else if(m_stick.getBButtonPressed()) {
        angle = angle - 10;
        //m_arm1.set(-0.5);
    }

    if (angle <= 0){
      angle = 0;
    }

    if (!limitSwitch.get()){
     armangle.reset();
    }

    if (m_stick.getXButton()) {
      m_shooter_bottom.set(0.7593);
      m_shooter_top.set(0.7593);
    } else {
      m_shooter_bottom.set(0);
      m_shooter_top.set(0);
    }
  
    // if (armangle.get() < 0){
    //   armangle.reset();
    // }

    if(m_stick.getStartButton()) { // This is the command for the intake mode
      intake.set(1);
      
    } 
    else if(m_stick.getYButton()){
      intake.set(-1);
    }
    else {
      intake.set(0);  
    }

    System.out.println(armangle.get());
    if (armangle.get() > angle){
      m_set = -m_pidControllerDown.calculate(armangle.get(), angle);
    }
    else if(armangle.get() < angle){
      m_set = -m_pidController.calculate(armangle.get(), angle);
    }
    m_arm1.set(m_set);
    // System.out.println(limitSwitch.get());
    System.out.println(m_pidController.calculate(armangle.get(), angle));
  }
}
