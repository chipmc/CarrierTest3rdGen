# Carrier Test For 3rd Generation Devices

This project is focused on automating the testing of the new 3rd Generation Carrier board.  

It will run through a series of tests and output the results to the Particle Console

The tests performed are:

1. - Test i2c bus and FRAM functionality
2. - Test the TMP-36 for temperature
3. - Test the User Switch - Requires physical Press
4. - Test that the RTC is keeping time
5. - Test an alarm on the RTC
6. - Test Battery charging - Can take a while based on state of charge
7. - Test Watchdog timer interval - Takes an hour
8. - Test Deep Sleep functionality

You can learn more about the carrier board and its development on the Particle Commmunity.

https://community.particle.io/t/boron-xenon-argon-carrier-for-outdoor-applications/48750?u=chipmc

