Welcome to the repository of my group's submission for COE 538 Final Project - Autonomous Maze Navigation.

This project involved having a robot called the EEBot navigate through a maze by itself using a path of tape as a guide through various turns and dead-ends. 

The navigation software was done using the following concepts: 
  - Line Sensor Detection: The bottom of the robot had pairs of LEDs and photodetectors that were used to detect how well it was aligned with the path and used the feedback from the sensors
                           to correct itself to remain on the path of tape.
    
  - Finite State Machine: A finite state machine design pattern was used to determine how and when the robot was to perform actions such as turning, reversing, and line tracking.
                          Each state of the robot changed its behaviour and would change based on an input that could either be a signal from the bumpers/sensors or a certain amount of time has passed.
    
      - The state machine used a dispatcher that made programming significantly easier. From the main loop, the dispatcher subroutine would read the current state of the robot
        and jump to the corresponding subroutine that performed the required action.
        
  - Right-hand-on-wall: This technique was used for its reliability and simplicity as the maze navigated was simple, had no loops, and the robot began from the start.

Disclaimer:
A few bumps and issues that our group faced when developing this software.
  - Our robot was very slow! At the time, we had no courses or knowledge of control theory, so our line tracking was simple but also very slow. This was because the corrections the robot made were in static preset distances and the margins for staying on the line were narrow.
  - Turn timings need to be tweaked. The robots that were used were old and the behaviour of the photodetectors and motors would vary a lot based on the battery voltage.
