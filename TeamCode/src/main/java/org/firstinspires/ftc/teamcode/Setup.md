Change the drive encoders to the correct direction then do the .forwardTicksToInches(-61.7)
//            .strafeTicksToInches(-2.08)
//            .turnTicksToInches(-0.02);

then do the DO the localization Test
// Run the Tuning OpMode, then navigate under to Localization Test
//On your computer, connect to your robot's Wi-Fi, and navigate to Panels or the FTC Dashboard. Panels is accessible at the ip address 192.168.43.1:8001 when connected to robot wifi.
//You should see the robot's position on the field.
//Observe the movements, make sure moving forward increases x and strafing left increases y.


then do  .xVelocity()
//            .yVelocity();

then do     .forwardZeroPowerAcceleration(deceleration)
//            .lateralZeroPowerAcceleration(deceleration);






PIDS


Setup
Open Panels. If you haven't used Panels before, you can read the documentation on the Panels.
On your Driver Hub or Driver Station, and connect a gamepad to it. Make sure to press "start" + "a" on the gamepad.
Select the Tuning Opmode. Use your gamepad to select the Manual folder. Then, select Translational Tuner.
Run the run the OpMode.
Note that while running the Translational Tuner OpMode, the robot will stay in place. This is intentional.


https://youtu.be/qe2eo_Mhtes



Push the robot left or right at varying amounts and observe how the robot corrects back to its starting position.
Adjust the PIDF constants (coefficientsTranslationalPIDF) in the Tuning-> Follower -> Constants section of Panels Configurables to ensure that the robot can accurately correct back to its starting position with minimal oscillations. For example, if the robot has too many oscillations while correcting back to its starting position, try lowering the P value. On the other hand, if the robot corrects back to its starting location too slowly, increase the P value. If you do not have prior experience with tuning PIDFs, we recommend that you check out the resources provided at the bottom of the PIDF Tuners page to learn more about tuning these.
If you have a dual PIDF system enabled, it is recommended to first tune the main PIDF, coefficientsTranslationalPIDF, to ensure that the robot can correct from large errors and bring it within the secondary PID's range. Then, tune the secondary PIDF, coefficientsSecondaryTranslationalPIDF, so the robot can smoothly correct from smaller errors and minimize oscillations.


After adjusting a value in Panels, hit "enter" in order to save it and cause the robot to correct differently. However, any values you modify through Panels are not saved into your code! In order to transfer the values you just tuned on Panels into your code, go to the Update Tuned Values section.

If additional feedforward is needed, use the feedforward term directly in the coefficientsTranslationalPIDF and/or coefficientsSecondaryTranslationalPIDF if you are using dual PID.
The feedforward term applies a minimum power output to the motors to compensate for the friction between the motors, wheels, and the ground.
To tune the feedforward, set all other PIDF values to 0 and increase the Feedforward value up until the robot starts moving/jittering.



Update Tuned values Into Your Code
Once you are satisfied with your translationalPIDF values, head over to the Constants file, and navigate to the FollowerConstants instantiation.
Navigate to or add the line .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
Update the parameters in new PIDFCoefficients(0.1, 0, 0.01, 0) with the translationalPIDFCoefficients values, P, I, D, F, you tuned on Panels in that order.
If you are using the dual PIDF system, add the line .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0)) and update the secondaryTranslationalPIDF values you tuned on Panels.










Heading PIDF
Setup
Open Panels. If you haven't used Panels before, you can read the documentation on the Panels Configurables.
On your Driver Hub or Driver Station, select the Tuning Opmode and then choose HeadingTuner.
Ensure that the timer for autonomous OpModes is disabled. Otherwise, the OpMode will automatically stop after 30 seconds.
Run the run the HeadingTuner autonomous OpMode.
Note that while running the Heading Tuner OpMode, the robot will stay in place. This is intentional.


https://youtu.be/-7M8puRdnfA


Turn the robot left or right at varying amounts and observe how the robot turns back to its starting heading.
Adjust the PIDF constants (coefficientsHeadingPIDF) in the Tuning-> Follower -> Constants section of Panels Configurables to ensure that the robot can accurately correct back to its starting position with minimal oscillations. For example, if the robot has too many oscillations while correcting back to its starting position, lower the P value. On the other hand, if the robot corrects back to its starting location too slowly, increase the P value. If you do not have prior experience with tuning PIDFs, we recommend that you check out the resources provided at the bottom of the PIDF Tuners page to learn more about tuning these.
If you have a dual PIDF system enabled, it is recommended to first tune the main PIDF, coefficientsHeadingPIDF, to ensure that the robot can smoothly correct back from large errors. Then, tune the secondary PIDF, coefficientsSecondaryHeadingPIDF, so the robot can smoothly correct from smaller errors.

Warning!

After adjusting a value in Panels, hit "enter" in order to save it and cause the robot to correct differently. However, any values you modify through Panels are not saved into your code! In order to transfer the values you just tuned on Panels into your code, go to the Update Tuned Values section to learn more.

Feedforward Adjustments (Optional)
If additional feedforward is needed, use the feedforward term directly in the coefficientsHeadingPIDF and/or coefficientsSecondaryHeadingPIDF if you are using dual PID.

The feedforward term applies a minimum power output to the motors to compensate for the friction between the motors, wheels, and the ground.
To tune the feedforward, set all other PIDF values to 0 and increase the Feedforward value up until the robot starts moving/jittering.
Update Tuned values Into Your Code
Once you are satisfied with your headingPIDF values, head over to the Constants file, and navigate to the FollowerConstants instantiation.
Navigate to or add the line .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
Update the parameters in new PIDFCoefficients(0.1, 0, 0.01, 0) with the headingPIDFCoefficients values, P, I, D, F, you tuned on Panels in that order.
If you are using the dual PIDF system, add the line .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0)) and update the secondaryHeadingPIDF values you tuned on Panels.

Then do Drive
The Drive PIDF manages acceleration and braking along a path, ensuring smooth motion and minimizing overshoot.

Setup
Open Panels. If you haven't used Panels before, you can read the documentation on the Panels Configurables.
On your Driver Hub or Driver Station, and connect a gamepad to it. Make sure to press "start" + "a" on the gamepad.
Select the Tuning Opmode. Use your gamepad to select the Manual folder. Then, select Drive Tuner.
Run the run the OpMode.
Warning!

Immediately after running the Drive Tuner Opmode, the robot will move straight back and forth 40 inches. Make sure you have enough space before running this opmode. You can adjust the distance the robot drives back and forth through Panels.
The robot WILL move laterally and/or change its heading. This is expected behavior since the translational and heading PIDFs are not activated. If both translational and heading PIDFs are tuned, consider using the Line Tuner to ensure the robot stays on the line.
Tuning Process
Setting the BrakingStrength
Before tuning the Drive PIDF, we will need to set the BrakingStrength. Head over to the deceleration page to learn more about it.

The BrakingStrength you set in the Constants class will be the default BrakingStrength for all paths the robot follows.

Observe how the robot moves back and forth through its path.
Adjust the PIDF constants (coefficientsDrivePIDF) in the Tuning-> Follower -> Constants tab of Panels Configurables to ensure that the robot smoothly and accurately drives straight back and forth.
Tuning Tips

PedroPathing does not activate heading and translational PIDF correction for drive tuning. If you would like to test all three of them, navigate yourself to the Line Tuner in the Manual folder. Use it to adjust BrakingStrength, path constraints and making sure all PIDFs are working well together.
Increasing your drive PIDF will make the robot move more quickly along the path, at the risk of more overshoot at the end of the path.
Decreasing your drive PIDF will make the robot move more slowly and reduce the overshoot at the end of the path.
Adjusting the BrakingStrength can significantly help manage how smoothly the robot decelerates as it reaches the end of its path.
If the robot drives quickly during the middle of the path but abruptly slows down as it reaches the end of the path, this may be caused by the transition between the main and secondary PIDs. This problem may also be addressed through lowering the BrakingStrength.
If you do not have prior experience with tuning PIDFs, we recommend that you check out the resources provided at the bottom of the PIDF Tuners page to learn more about tuning these.

If you have a dual PIDF system enabled, it is recommended to first tune the main PIDF, coefficientsDrivePIDF before tuning the secondary PIDF, coefficientsSecondaryDrivePIDF.

Warning!

After adjusting a value in Panels, hit Enter in order to save it and cause the robot to correct differently. However, any values you modify through Panels are not saved into your code! In order to transfer the values you just tuned on Panels into your code, go to the Update Tuned Values section to learn more.

Braking Start (Optional)
Braking Start determines when the robot starts braking when global deceleration (deceleration upon the entire PathChain) is active. Braking Start can be adjusted just like BrakingStrength.

Feedforward Adjustments (Optional)
If additional feedforward is needed, use the feedforward term directly in the coefficientsDrivePIDF and/or coefficientsSecondaryDrivePIDF if you are using dual PID.

The feedforward term applies a minimum power output to the motors to compensate for the friction between the motors, wheels, and the ground.
To tune the feedforward, set all other PIDF values to 0 and increase the Feedforward value up until the robot starts moving/jittering.
Kalman Filter Adjustments (Optional)
The drive PID uses a Kalman filter to smooth error responses:

Model Covariance: Default is 6.
Data Covariance: Default is 1.
A higher model covariance to data covariance ratio will cause the filter to rely on the previous output rather than the data (raw drive error).
A lower model covariance to data covariance ratio will cause the filter to rely on the data rather than the previous output
To modify these values, add the line FollowerConstants.driveKalmanFilterParameters(6, 1) in your Constants file and replace the parameters with the desired ones.
The drive PID also has a filter such that the derivative term is a weighted average of the current derivative and the previous derivative.

The default time constant T for the drive filtered PID is 0.6, meaning that the derivative output is 0.6 times the previous derivative plus 0.4 times the current derivative.
You can modify this value by changing the fourth parameter in the drivePIDFCoefficients(P, I, D, T, F).
Feel free to experiment with these settings for optimal performance.

Update Tuned values Into Your Code
Once you are satisfied with your drivePIDF values, head over to the Constants file, and navigate to the FollowerConstants instantiation.
Navigate to or add the line .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0))
Update the parameters in new FilteredPIDFCoefficients(P, I, D, T, F) with the drivePIDFCoefficients values, P, I, D, F, you tuned on Panels.
If you are using the dual PIDF system, add the line .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01)) and update the secondaryDrivePIDF values you tuned on Panels.






The centripetal force correction enables the robot to accurately follow curved paths.

Setup
Open Panels. If you haven't used Panels before, you can read the documentation on Panels Configurables.
On your Driver Hub or Driver Station, select the Tuning Opmode, navigate to Manual and then choose CentripetalTuner.
Ensure that the timer for autonomous OpModes is disabled. Otherwise, the OpMode will automatically stop after 30 seconds.
Run the run the CentripetalTuner autonomous OpMode.
Warning!

Immediately after running the Centripetal Tuner Opmode, the robot will move forward and left 20 inches in a curved path. Make sure you have enough space before running this opmode.
You can adjust the distance the robot drives back and forth through Panels.
Tuning Process
Follow this video to help you tune the centripetal scalar:
https://youtu.be/728GLkqy9yY

Observe the robot’s path:

If the robot corrects towards the inside of the curve, decrease centripetalScaling.
If the robot corrects towards the outside of the curve, increase centripetalScaling. Adjust the value of centripetalScaling within the Tuning-> Follower -> Constants section in Panels.
Update Tuned values Into Your Code
Once you are satisfied with your centripetalScaling, head over to the Constants file.
Navigate to the line .centripetalScaling(0.005) under followerConstants. If you don't have this line, feel free to add it yourself.
Update the parameters in .centripetalScaling(0.005) with the centripetalScaling value you tuned.
Troubleshooting
If you have any problems, see the troubleshooting page.





Tests
Validating your tuning

To validate your tuning, it is prudent to try running at least one of these tests. They are runnable via the Tuning class and using the gamepad to the Tests folder.

Line
Line Test is used to analyze the follower's capability of driving with all of the PIDFs active at once. This helps determine if any PIDF(s) need adjusting. The follower will drive 48 inches forward (two tiles) and then back to the initial position. It will loop this action.

Triangle
Triangle Test is used to ensure that the follower is capable of straight line interpolation. This helps determine if any PIDF(s) need adjusting. The follower will drive in a triangle path, looping infinitely.

Circle
Circle Test is used to ensure that the follower is capable of curved path following. This helps determine if any PIDF(s) or if the Centripetal Scaling needs adjusting. The follower will drive in a circle path always facing the center, looping infinitely.