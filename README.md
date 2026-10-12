# Arm Demonstrations for UR3e arm, MRRP lab

This branch has rock paper scissors as an option. There is old code from Nathan Sexton and the Colman group making up 90% of this branch since this was my first project with the arm and borrowed a lot from them.

## Instructions:

### 1. Clone this repository 

```
git clone https://github.com/MarkTheHumble/Arm-Demonstrations.git
```

### 2. Install pixi if pixi is not installed on your machine

```
curl -fsSL https://pixi.sh/install.sh | sh
```

### 3. Build the project

```
pixi run build
```

### 4. Power the robot arm

Silver power button on the tablet connected to the machine running the arm

### 5. Have the arm listening for remote instructions

Near the top right of the tablet screen, there will be a tablet logo saying "local". Tap that logo. After, select "remote"

### 6. Physically connect the arm to the computer

There's an ethernet cable under each robot arm. Connect it to your computer's ethernet port.  

### 7. Digitally connect the arm to the computer

If you're using one of the lab machines near it or just Linux in general, select "UR3_A" as your network for your wired connection, because that is never done automatically. This has to take place after step 6.


### 8. Start communicating with the arm

```
pixi run launch-lan
```  

You should see RViz appear. If the arm on your screen is in a different position than the robot you're trying to communicate with, something is wrong. Check that the ethernet cable is fully inserted and at least occasionally blinking. Check that the network you're connected to is the network for the arm and not the internet connection for the lab. Check that the tablet for the arm is set to "remote" control instead of "local". The ethernet cable is currently being held together by tape as of October 2026, so that could also be a point of failiure soon.


### 9. Launch the game

inside of a different shell, with the shell that performed "pixi run lan" still running,

```
pixi run rps
```

Spend the first 6.5 seconds of the game aligning the 8x6 board with the temporary red lines that should be going through the vertical and horizontal centers of the checkers board. The game is currently expecting blue pieces on the arm side and pink pieces on the players side.

### Aftermath

Set the arm back to "home" position.  
Shut down the arm.