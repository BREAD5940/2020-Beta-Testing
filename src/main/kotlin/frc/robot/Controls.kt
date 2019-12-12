package frc.robot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.climb.ClimbSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.TestTrajectory
import frc.robot.subsystems.superstructure.*
import frc.robot.subsystems.superstructure.Elevator
//import frc.robot.subsystems.superstructure.ElevatorPresets
//import frc.robot.subsystems.superstructure.elevatorPresets
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import org.ghrobotics.lib.wrappers.hid.*
import frc.robot.subsystems.superstructure.Elevator as Elevator1

object Controls {


    class XboxController {


        val driverControllerLowLevel = XboxController(0)
        val driverFalconXbox = driverControllerLowLevel.mapControls {


            // button(kB).changeOn(DriveSubsystem.followTrajectory(TestTrajectory.trajectory)).changeOn {
            // DriveSubsystem.odometry.resetPosition(TestTrajectory.trajectory.states[0].poseMeters)
            //}
                //^^^ USE semi colin or comma to shorten ^^^
            //TODO finish all button layouts and follow previos project.
            //top line is on press and bottom is when you let go of the button
            state({ driverControllerLowLevel.getRawButton(10) }) {
                button(kA).changeOn { ArmPresets.cargoLow() }
                button(kX).changeOn { ArmPresets.cargoMid() }
                button(kY).changeOn { ArmPresets.cargoHigh() }
                if(ArmPresets.hab3 or ArmPresets.hab2){
                    button(kBumperLeft).changeOn { ClimbSubsystem.intakeWheels.setDutyCycle(1.0) }.changeOff { ClimbSubsystem.intakeWheels.setNeutral() }
                    button(kBumperRight).changeOn { ClimbSubsystem.intakeWheels.setDutyCycle(1.0) }.changeOff { ClimbSubsystem.intakeWheels.setNeutral() }
                }
                else {
                    button(kBumperLeft).changeOn { Intake.cargoIntake() }.changeOff { Intake.stop() }
                    button(kBumperRight).changeOn { Intake.cargoOutake() }.changeOff { Intake.stop() }
                }
            }
            state({ !driverControllerLowLevel.getRawButton(10) }) {
                button(kA).changeOn { ArmPresets.hatchLow() }
                button(kX).changeOn { ArmPresets.hatchMid() }
                button(kY).changeOn { ArmPresets.hatchHigh() }
                if(ArmPresets.hab3 or ArmPresets.hab2){
                    button(kBumperLeft).changeOn { ClimbSubsystem.intakeWheels.setDutyCycle(-1.0) }.changeOff { ClimbSubsystem.intakeWheels.setNeutral() }
                    button(kBumperRight).changeOn { ClimbSubsystem.intakeWheels.setDutyCycle(-1.0) }.changeOff { ClimbSubsystem.intakeWheels.setNeutral() }
                }
                else {
                    button(kBumperLeft).changeOn { Intake.hatchIntake() }.changeOff { Intake.stop() }
                    button(kBumperRight).changeOn { Intake.hatchIntake() }.changeOff { Intake.stop() }
                }
            }
            button(kB).changeOn { ArmPresets.stowed() }
            pov(90).changeOn {ArmPresets.hab3Prep()}
            pov(270).changeOn {ArmPresets.hab2Prep()}
            pov(0).changeOn {ArmPresets.habYEET()}
            pov(180).changeOn {ArmPresets.noHab()}

        }

            val operatorWPIJoystick = XboxController(1)
            val operatorFalconXbox = operatorWPIJoystick.mapControls {
            }


            fun update() {
                driverFalconXbox.update()
                operatorFalconXbox.update()
            }
        }

        private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen; +InstantCommand(Runnable(block)) }
        fun update() {
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        }
    }

