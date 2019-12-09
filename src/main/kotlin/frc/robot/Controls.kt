package frc.robot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.TestTrajectory
import frc.robot.subsystems.superstructure.*
import frc.robot.subsystems.superstructure.Elevator
//import frc.robot.subsystems.superstructure.ElevatorPresets
//import frc.robot.subsystems.superstructure.elevatorPresets
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.radians
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
            //Presets TODO map controls for presets (elevator, wrist, proximal side done (still needs to be checked))
            //top line is on press and bottom is when you let go of the button
            button(kA).changeOn { ElevatorPresets.elevatorGoToPreset(height = 10.inches) ; Wrist.wristPreset(WristRadian = 0.4.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.6.radians) }.changeOff {
                ElevatorPresets.elevatorGoToPreset(height = 0.inches) ; Wrist.wristPreset(WristRadian = 0.0.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.0.radians) }
            button(kB).changeOn { ElevatorPresets.elevatorGoToPreset(height = 10.inches) ; Wrist.wristPreset(WristRadian = 0.4.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.6.radians) }.changeOff {
                ElevatorPresets.elevatorGoToPreset(height = 0.inches) ; Wrist.wristPreset(WristRadian = 0.0.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.0.radians) }
            button(kX).changeOn { ElevatorPresets.elevatorGoToPreset(height = 20.inches) ; Wrist.wristPreset(WristRadian = 0.4.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.6.radians) }.changeOff {
                ElevatorPresets.elevatorGoToPreset(height = 0.inches) ; Wrist.wristPreset(WristRadian = 0.0.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.0.radians) }
            button(kY).changeOn { ElevatorPresets.elevatorGoToPreset(height = 20.inches) ; Wrist.wristPreset(WristRadian = 0.4.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.6.radians) }.changeOff {
                ElevatorPresets.elevatorGoToPreset(height = 0.inches) ; Wrist.wristPreset(WristRadian = 0.0.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.0.radians) }
            button(kA).changeOn { ElevatorPresets.elevatorGoToPreset(height = 30.inches) ; Wrist.wristPreset(WristRadian = 0.4.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.6.radians) }.changeOff {
                ElevatorPresets.elevatorGoToPreset(height = 0.inches) ; Wrist.wristPreset(WristRadian = 0.0.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.0.radians) }
            button(kB).changeOn { ElevatorPresets.elevatorGoToPreset(height = 30.inches) ; Wrist.wristPreset(WristRadian = 0.4.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.6.radians) }.changeOff {
                ElevatorPresets.elevatorGoToPreset(height = 0.inches) ; Wrist.wristPreset(WristRadian = 0.0.radians) ; Proximal.ProximalPreset(ProximalRadian = 0.0.radians) }


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
    fun update() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}
