Here is work done by Mentor Josh, in an attempt to learn how to program an FRC robot.

I (Josh) have decided to do this by reprogramming our 2025 robot, One in a Krillion, using a State-Machine control scheme. This is a still work in progress, and as the robot as been scrapped, all code is untested.
Here is progress so far:
src\main\java\frc\robot\
  commands - Not touched yet
  
  constants - Mostly base values from last year's robot, tuning would need to be done, as well as confirming port values
  
  config - Again, mostly taken from base code, double checking would be required
  
  subsystems
  
    arm - 'Done' (as much as can be with robot)
    
    climb - Not ready, just basic layout
    
    drive - 'Done'
    
    elevator - 'Done'
    
    endEffector - Everything laid out, except for beambreaks for coral/algae detection
    
    hpintake - Not ready, just basic layout
    
    vision - 'Done'
    
  superstructure - gotten as far as I can without all subsystems implemented
  
  util - Done
  
  LocalADStarAK.java - Code from 1732 to use QuestNav with a pathfinder
  
  Main.java - 'Done'
  
  Robot.java - needs touching up with pathfinding and logging
  
  RobotContainer.java - gotten as far as I can without all subsystems implemented
  
  RobotState.java - Not really touched from 2910's code base
  
  
