# Incremental encoder triggered speed control loop [STM32L432KC]
The first two chapters were envisaged to encourage you to experiment with [pulse counting](https://github.com/ufnalski/bmw_steering_column_adjust_motor_l432kc) and [pulse timing](https://github.com/ufnalski/audi_a4_tailgate_lift_motor_l432kc) as speed calculation methods. The third chapter of the incremental encoder trilogy is devoted to encoder event triggered control system. The idea has been signaled [here](https://github.com/ufnalski/audi_a7_stabilus_powerise_l432kc) in the what next section. Now it is high time we experimented with such a system. Let us investigate its benefits and challenges. A low-CPR (counts per revolution) sensor is used to explore limits of the closed-loop dynamics. A proportional-integral (PI) speed controller is deployed to constitute a basic example of such a feedback control system.

![Incremental encoder triggered speed control loop in action](/Assets/Images/encoder_triggered_control_in_action.jpg)
![Incremental encoder triggered speed control loop (serial plotter)](/Assets/Images/encoder_triggered_control_serial_plotter.jpg)

> [!TIP]
> Variable sampling period introduces variable delay. That delay should be taken into account when tuning the controller. The resulting gains of the controller are adjusted on the fly. You might call it an adaptive system, but in fact this is to mimic the controller with constant [gains](http://www.ufnalski.edu.pl/proceedings/misc/Naslin_vs_Kessler_BEAMER.pdf) running at a constant sampling time.

> [!IMPORTANT]
> The encoder triggered control system is bound to fail at zero speed - no input capture interrupt ```c HAL_TIM_IC_CaptureCallback()``` means no controller code execution. To fix that, an upper limit of the update period has to be introduced for low speeds. Note the output capture interrupt ```c HAL_TIM_OC_DelayElapsedCallback()```. The control system switches between the encoder event triggered one and the fixed sampling time one. The transition point is defined by ```PULSE_TIMING_ARR```.

> [!NOTE]
> The low-CPR encoder is used to highlight the effect of delayed information about the actual speed on the control system. 

# Missing files?
Don't worry :slightly_smiling_face: Just log in to MyST and hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32L4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Readings
* [Speed Estimation Algorithm with Specified Bandwidth for Incremental Position Encoder](https://ieeexplore.ieee.org/document/7827811) (Alecksey Anuchin at al.)
* [Synchronous Constant Elapsed Time Speed Estimation Using Incremental Encoders](https://ieeexplore.ieee.org/document/8770285) (Alecksey Anuchin at al.)
* [On Speed Estimation from Incremental Encoders with Tunable Error Bounds](https://www.sciencedirect.com/science/article/pii/S2405896321017559) (Dario Savaresi at al.)

My experiments with low-CPR encoders indicate that the most challenging conditions are not created when the drive accelerates from zero to nominal speed. The accuracy of the speed estimation/calculation method plays only a moderate role when the controller operates in saturation. Moreover, the control system leaves the saturation when the speed is already far from low ones, and the information on speed can be updated frequently if an incremental encoder is considered. Reversing the drive is also not the most demanding scenario for a speed calculation algorithm. The most challenging conditions for a drive with a relatively low-CPR encoder are present when approaching zero or near-zero speed and then trying to stay at zero or near-zero speed. Let us do some edge condition testing (AKA boundary value analysis). For the zero speed there are no new pulses at all. By measuring time passed from the last pulse we can update (decrease) the speed estimate over time waiting for the next pulse to occur. This results in relatively slow settling time. And do not worry about the timer overflows - simply count them and add them to the formula. For example, for the 16-bit timer (counter) and the clock running at say 160 MHz you will get around 0.4 millisecond between overflows. You can then use a software counter (to form a 2-fold cascade) and extend the measurement range practically indefinitely if you wish/need. This is just to illustrate that you do not have to spare a 32-bit timer to get high-resolution measurements.

# Call to action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2025_dzien_popularyzacji_matematyki/Dzien_Popularyzacji_Matematyki_2025.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [bitluni's lab](https://www.youtube.com/@bitluni), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), [Nikodem Bartnik](https://www.youtube.com/@nikodembartnik), [Stuff Made Here](https://www.youtube.com/@StuffMadeHere), [Mario's Ideas](https://www.youtube.com/@marios_ideas), [Aaed Musa](https://www.aaedmusa.com/), [Haase Industries](https://www.youtube.com/@h1tec), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them! Promote [README-driven learning](http://ufnalski.edu.pl/proceedings/sene2025/Ufnalski_PE_formatted_SENE_2025.pdf) :sunglasses:

> [!WARNING]
> Feedback control systems - do try them at home :grey_exclamation:

220+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
