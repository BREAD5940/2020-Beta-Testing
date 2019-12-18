
package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.controller.ElevatorController
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated

        // + for subsystems
        +DriveSubsystem

        SmartDashboard.putData(CommandScheduler.getInstance())

        super.robotInit()
    }

    override fun autonomousInit() {
    }

    override fun teleopPeriodic() {
    }

    override fun robotPeriodic() {
        Controls.update()
        Elevator.updateState()
        Elevator.useState()
    }

    override fun disabledInit() {
    }

    override fun teleopInit() {
    }
}

fun main() {
    Robot.start()
}
