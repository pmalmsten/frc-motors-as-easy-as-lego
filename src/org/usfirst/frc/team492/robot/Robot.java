/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team492.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controlledmotors.SoftwareControlledMotor;
import edu.wpi.first.wpilibj.controlledmotors.VelocityControlledMotor;
import edu.wpi.first.wpilibj.drive.VelocityControlledDifferentialDrive;
import edu.wpi.first.wpilibj.mechanisms.SoftwareControlledElevator;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
    private VelocityControlledDifferentialDrive m_myRobot;
    private SoftwareControlledElevator m_elevator;

    private Joystick m_leftStick;
    private Joystick m_rightStick;

    @Override
    public void robotInit() {
        final VelocityControlledMotor leftMotor = SoftwareControlledMotor.builder()
                .withSpeedController(new Spark(0))
                .withEncoder(new Encoder(0,1))
                .withMaxSpeedInSensorUnitsPerSecond(3000) // max ticks per second
                .build();

        final VelocityControlledMotor rightMotor = SoftwareControlledMotor.builder()
                .withSpeedController(new Spark(1))
                .withEncoder(new Encoder(2,3))
                .withMaxSpeedInSensorUnitsPerSecond(3000) // max ticks per second
                .build();

        m_myRobot = new VelocityControlledDifferentialDrive(leftMotor, rightMotor);

        m_elevator = new SoftwareControlledElevator(
                new Spark(2),
                new Encoder(4, 5),
                0.1, // Gravity compensation torque
                1 / 1500, // Inches per tick
                0, // Sensor value at zero height
                3234 // ticks per second at max motor RPM
        );

        m_leftStick = new Joystick(0);
        m_rightStick = new Joystick(1);
    }

    @Override
    public void teleopPeriodic() {
        m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

        m_elevator.setPosition(10); // Inches
    }
}
