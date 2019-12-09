package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.TestTrajectory
import frc.robot.subsystems.superstructure.ElevatorPresets
import frc.robot.subsystems.superstructure.Intake
import frc.robot.subsystems.superstructure.Superstructure
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.ghrobotics.lib.wrappers.hid.*
import frc.robot.subsystems.superstructure.Elevator as Elevator1

object Controls {
    class XboxController : FalconCommand(Elevator1){
        val driverControllerLowLevel = XboxController(0)
        val driverFalconXbox = driverControllerLowLevel.mapControls {

            //button(kX).changeOn(InstantCommand(Runnable { "Hullo!" }))
            // button(kB).changeOn(DriveSubsystem.followTrajectory(TestTrajectory.trajectory)).changeOn {
            // DriveSubsystem.odometry.resetPosition(TestTrajectory.trajectory.states[0].poseMeters)
            //}
//TODO finish all button layouts and follow previos project.
            button(kA).changeOn { Intake.cargoIntake() }.changeOff { Intake.stop() }
            button(kB).changeOn { Intake.cargoOutake() }.changeOff { Intake.stop() }
            button(kX).changeOn { Intake.hatchIntake() }.changeOff { Intake.stop() }
            button(kY).changeOn { Intake.hatchIntake() }.changeOff { Intake.stop() }
            //Presets TODO map controls for presets (elevator side done)
            button(kA).changeOn { Intake.cargoIntake() }.changeOff { ElevatorPresets.Cargo. }
            button(kB).changeOn { Intake.cargoOutake() }.changeOff { Elevator1.master.setPosition(33.inches) }
            button(kX).changeOn { Intake.hatchIntake() }.changeOff { Intake.stop() }
            button(kY).changeOn { Intake.hatchIntake() }.changeOff { Intake.stop() }
            button(kA).changeOn { Intake.cargoIntake() }.changeOff { Intake.stop() }
            button(kB).changeOn { Intake.cargoOutake() }.changeOff { Intake.stop() }


        }

        val operatorWPIJoystick = XboxController(1)
        val operatorFalconXbox = operatorWPIJoystick.mapControls {
        }

        fun update() {
            driverFalconXbox.update()
//        operatorFalconHID.update()
            operatorFalconXbox.update()
        }
    }

    private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen; +InstantCommand(Runnable(block)) }
}
