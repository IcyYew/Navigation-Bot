# Navigation-Bot
**Collaborators:** Michael Geltz, Joshua Dwight, Nathan Tegeler, Bennet Palkovic

**Platform:** Tiva TM4C123GH6PM Microcontroller

**Objective:** Make a roomba autonomously navigate an obstacle course, find and "rescue" the objective and then exit the course.

## Description
In this course our final project was to make a robot(Roomba fitted with a Tiva TM4C123GH6PM Microcontroller). With additional peripherals controlled by the microcontroller, sonar and infared distance measuring capabilities, a servo allowing for 360 degree movement of the sensors, and an LCD screen. Navigate an obstacle course defined by white tape with an objective (in our case the thinnest PVC pipe) and obstacles randomly placed around the defined area by the teaching assistants. These were PVC of varying heights and diameters and also a "void" or "hole" in the floor defined by black tape. For our purposes we wanted out robot to fully autonomously navigate the course without touching any of the obstacles or falling into the "void" until it reached the objective, after it reached the objective successfully it must then navigate itself out of the course safely. 

## Specifics Of Our Project
- **Navigation**
  * Sonar sensor attached to a server was used for long distance obstacle/object detection, with support from the infared scanner to calculate actual object size negating distance distortion
  * Onboard bump sensors of the roomba allowed for us to detect and deal with the shortest obstacles -- beneath the sensors scanning height
  * Onboard color sensor of the roomba allowed for us to detect the field perimeter and avoid entering the "void"
  * Based upon above information we derived an object algorithm "objectAlgo_v3.c" to successfully find our objective and navigate the course
- **Additionals**
  * Breadboard hooked up to the "free" ports of our microcontroller to activate a specific color of LED depending on what state the robot was in. Yellow for when the bot is in navigation mode, red for while the bot is "rescuing" the objective, and green for when the objective was successfully "rescued".
  * Onboard sound system reworked to play a snippet of *Undertale - Megalovania* after a successful "rescue"
 
## Skills Developed
- **Microcontroller & Peripherals:**
    * **Comprehending Datasheets:** Gained expertise in reading and digesting datasheets for the microcontroller and peripherals allowing for proper configuration of registers and ports.
    * **Peripheral & Microcontroller Communication:** Utilized GPIO, ADC, UART and PWM to control the sensors, sounds, and LED system.

- **Embedded Systems Programming:**
    * **Real-Time Events:** Ensured the robot was timely in communicating sensor information and then acting upon a calculated safe path.
    * **Interrupts:** Utilized volatile variables with interrupt service routines to handle asynchronous events.
    * **Optimizations:** Learned to work with the robots limited memory by shrinking variable sizes, where possible and allowing for allocation sacrifices to happen where they were needed.
    * **Low-Level Programming:** Wrote code to directly manipulate registers to control the peripherals and onboard features.
    * **Etc...**
