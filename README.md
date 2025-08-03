# Pursuer_Evader_Games

This repository contains all the code and simulations developed for my project on Pursuer-Evader Differential Games.


## What is the basic idea behind these games?
There is a team of evaders, and a team of pursuers. Evaders want to reach a target point or a goal region. 
The pursuers want to capture as many evaders as possible before they reach the goal. We try to come up with optimal strategies for both the teams. 
It is normally assumed that there is no communication between the evaders and purusers.
Many times while strategizing for pursuers, we tend to treat the evaders as independent entities to make it easier.

## What are the real world applications of these games?
These concepts can be applied to:

Robotics: Multi-robot coordination and interception
Surveillance: Security systems and threat assessment
Autonomous Vehicles: Traffic management and collision avoidance
Military: Tactical planning and engagement strategies
Game Development: AI behavior for strategy games

## Project Structure

EV3_Controller - A ROS-based pipeline integrating an 8-camera OptiTrack system for real-time orientation and localization with LEGO EV3 robots. This closed-loop system provides continuous feedback, allowing robots to adjust their movements based on live positional data.

MEG_Python - Python simulations of pursuit-evasion scenarios. Explores strategy development for the pursuit team (1 pursuer vs. n evaders), combining weighted averages of optimal headings and nearest-evader pursuit tactics.

## Current Work
Investigating matching-based strategies for pursuers.
