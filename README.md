# EthicalViolence

The winning Zumo codebase from the Microsoft competition at EMF Camp 2016! 

You can see it in action winning the final of the competition here: https://twitter.com/liliankasem/status/761963212556361732

The demo script this is based off can [be found here](https://github.com/pololu/zumo-shield/blob/master/ZumoExamples/examples/SumoCollisionDetect/SumoCollisionDetect.ino).

## Getting Started

1. Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software)
2. Install the Libraries using [these instructions carefully](https://github.com/pololu/zumo-shield#software) - be warned - it's not a standard library install for some reason, you do have to read those closely. We do require LSM303, so install that too as per the section.
3. You're then ready to go. Load the .ino in the IDE and flash it to the Zumo.

## About the code

* The collision detection has been ripped out entirely. That's our secret. ;)
* Randomisation features heavily in this code and is initialised properly. What you can't predict, you can't program explicitly against.
* This specifically targets Zumo robots coded either with a single-shove strategy or a tweaked-default strategy.
  * Single-shove is avoided using a random initial shove avoid routine.
  * Ending up in a head-to-head shoving battle comes down to random chance or the individual robot's setup; it's really best avoided.
  * Tweaked-default is find the line, reverse, change angle, try again. We find those Zumos repeatedly hacking against an edge, and scoop them up sideways.
* The edge detection is pretty good, and it shouldn't run itself out of the arena. If it does, change the motor speed settings. You may be able to run this and simply outlive the others if they have their motors set up too fast.
