# Pursuer_Evader_Games

This repository contains all the code from my Project on Pursuer Evader Differential Games

## What is the basic idea behind these games?
There is a team of evaders, and a team of pursuers. Evaders want to reach a target point or a goal region. 
The pursuers want to capture as many evaders as possible before they reach the goal. We try to come up with optimal strategies for both the teams. 
It is normally assumed that there is no communication between the evaders and purusers.
Many times while strategizing for pursuers, we tend to treat the evaders as independent entities to make it easier.

## What are the real world applications of these games?
This framework can be applied to various real-world scenarios:

Robotics: Multi-robot coordination and interception
Surveillance: Security systems and threat assessment
Autonomous Vehicles: Traffic management and collision avoidance
Military: Tactical planning and engagement strategies
Game Development: AI behavior for strategy games

## Project Structure

EV3_Controller - Created this ROS based pipeline to integrate the 8 Camera Optitrack System(for live orientation and positioning) and the EV3 devices. It is a closed loop system where EV3 bots receive feedback from the positioning system and act upon it.
MEG_Python - We try to come up with strategies for the pursuit team in 1 pursuers vs n evaders, using weighted average of optimal headings and closest evader pursuit methods. The code for python simulation is given.

## Current Work
Looking into Matching based strategies for pursuit team to capture evaders.
