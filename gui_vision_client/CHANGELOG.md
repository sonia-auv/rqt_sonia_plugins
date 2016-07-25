# Change Log Vision Client

## BETA REALEASE 1

**Objective**

Basic functionnalities for test purpose

**Main Feature**

In this realease, the goal is to provide a very basic software,
usable but not advanced, that will allow user to test some fonctions of vision server

### Beta 1.0

**Features List**

1. Display the list of the executions
2. Display the list of the filters for an execution
3. Display the list of the parameters for a filter
4. Display a video rendering of the current (selected) execution

**Fixed Bug**

* Video rendering resizing
* Don't display anything if the executionList is empty
* Double free memory on application exit


### Beta 1.1

This release is a developeur version. It won't provide any user feature but will help developpeur team to build
a better solution in the future.

**Features List**

1. Changed conception for better code readability and evolutivity. This should be the definitive implementation
2. The QWidget in UI are now implemented in the logic code.
3. Each of the Widget that "contains" a component are now implementing ContainterWidget
4. Every class is now inheriting from QObject to ensure that you can emit and receive signals every where

## MAJOR RELEASE 1

**Objectives**

Basic developpment tools

**Main Feature**

After having implemented all the basics functionnnalities, the goal is to provide
a bunch of developpment tools that will improve the usage of vision server

### Minor Realease 1.0

**Features List**

1. Send changes on a parameter to vision server
2. Save the current state of a filter chain
3. Launch an execution from the client
4. Change the video rendering on change filter action

**Fixed Bug**

* The initial value of checkbox is wrong
* The Media list is incresing at each instance of vision client

### Minor Realease 1.1

**Features List**

Better display for parameters

1. Interval of values for slider
2. Display numeric value of int parameter
3. Description of parameters

### Minor Realease 1.2

**Features List**

1. [DONE] Record a video
2. [DONE] Stop an execution from the Client
3. [DONE] Recharger les exécutions en cours.
4. [DONE] Chain the positino of a Filter in a Filter Chain

## MAJOR RELEASE 2

**Objectives**

Advanced features for the vision client

**Main Feature**

Provide advanced features to allow user to take full advantage of the vision server.

### Minor Realease 2.0

**Features List**

Ajout d’un mode console

1. [X] Logger on terminal tab
2. [X] Feed text topic_result under topic_video

** Bug reported **

1. [X] Block parameter description editing
