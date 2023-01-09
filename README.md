# Turtlebot_reactive_behaviour
## 1. Introduction

This project focuses on the implementation of different reactive behaviour algorithms in a mobile robot. The task to be completed is wall following, in which the robot must move parallel to the wall, avoiding collision with it and maintaining a certain margin of distance. 
This task is performed using the open source robot simulator Webots [1]. In this simulator, two different scenarios are designed, in which the movement of the robot will be tested.
Several different methods are used to realise this reactive behaviour. The first one will be based on behavioural rules, the second one implements a PID that maintains a lateral distance to the wall and the third one uses different machine learning models to drive the robot.
The robot used is a Turtlebot 3 Burger [2]. This is a differential configuration robot equipped with a laser scanner that allows it to detect obstacles around it and will be used to track walls.
The following sections detail the different topics of the process that has been followed. The tracking implemented is designed for walls on the left side of the robot. The process for walls on the right side is analogous to that carried out, and the programme can be adapted in a trivial way.

## 2. Robot controllers

### 2.1 Ruled based controller
The first method developed for operating the robot implements a several states that can be reached according to the measurements taken from the sensors. 
The Turtlebot 3 robot has a laser scanner that provides a vector with 360 measurements [3], corresponding to the distance measurements in 360 degrees around the robot. With these measurements, obstacles around the robot can be detected, or as in this case, the wall to follow. 
The laser scanner is placed on the robot in such a way that the measurement starts right at the back of the robot, moving clockwise until the 360 degrees are completed. The following Figure 1 clarifies the orientation of the scanner and measurements of the 4 main faces of the robot, which correspond to the same position in the vector of distances.

![image](https://user-images.githubusercontent.com/101353583/211430500-0f68a7ae-ffc8-4dd2-be00-0687f8275b4e.png)

Figure 1. Turtlebot laser scanner orientation

In fact, only 3 different measurements are taken into account for the behavioural rules, which correspond to the angles (positions of the distances vector) indicated in the following table:

Table 1. Distance measurements taken

Measurement	Front	Corner	Side
Corresponding angle	180	135	90

The logic of the programme consists in using the measurements of these three angles and compare them with a threshold. This allows to analyse whether there is front, side or corner wall detection, and to act accordingly. Avoiding an impact with a front wall will be prioritised over side wall tracking, which is implemented by moving towards or away from it as necessary. 
The behaviour follows the following truth table:

Table 2. Behaviour truth table

Left side wall	Front Wall	Action
Doesn´t matter	Yes	Rotate right
No	No	Turn Left
Yes	No	Move Forward

To implement this behaviour, a set of functions are defined that will drive the robot according to the action to be performed, commanding the motors with the necessary speed commands.

### 2.2. PID controller
A wall tracking based on a control loop with a PID controller is implemented. 
This controller reacts to an error signal proportionally (it acts in proportion to the instantaneous error), integrally (it acts on the accumulation of error) and derivatively (it tries to keep the error constant). In this case, the error is the distance to the wall minus the desired distance setpoint. 
In a control loop, this regulation allows to act on a plant by means of actuators, which in this case is our robot. The control loop is closed by sensors, in this case the laser scanner.

![image](https://user-images.githubusercontent.com/101353583/211430685-10c29586-c0a5-496f-b322-1f7a6ca894e6.png)
 
Figure 2. Closed loop control system

The continuous-time equation for a PID is as follows [4]:
u_PID=K_P∙e(t)+K_I∙∫_(t=0)^t▒e(t)dt+K_D  (de(t))/dt
This continuous-time controller, in order to be implemented in a discrete system, such as a robot computer, must be discretised.
Two different approaches are carried out, one using a simple PID and the other one discretising a continuous PID by means of a Z-transform. 
For both cases, the integral performance is not of interest and will not be used. The steady state behaviour is adequate and it is observed that adding a pole to the system (in this case, an integrator) makes it marginally stable, going into permanent oscillations. 
Consequently, a zero gain is set for the integral action, so we are really dealing with a PD regulator.
The PID actuations are used to compute the speed references of the motors. The measurements used to calculate the error signal are obtained by minimizing a set of distances obtained from the laser scanner.
PID Simple
A simple PID is implemented, for which the differential action is computed as a subtraction and the integral action as a cumulative sum. 
The equation used is:

*correction = Kp * error + Kd * rateError + Ki * cumError*

Where:

	- error: is the subtraction between the lateral distance margin and the sensor measurement at 90º (left).
	- rateError: is the subtraction between the current (instant k) and previous (instant k - 1) error signal.
	- cumError: It is the accumulated sum of the current error with the previous ones. 
  
Different gains are tested, finally obtaining a satisfactory response with Kp = 30, Kd= 12 and Ki = 0.
Discretized PID 
In addition to the widely used simple discrete PID, the discretisation of a continuous PID is carried out using the Tustin method and the inverse Z-transform (developed in the Appendix). The equation in differences is obtained, which can be implemented in a discrete controller such as the one used by this robot. The difference equation of the controller is:

*u_k=u_(k-1)+K_p (e_k-e_(k-1) )+(K_p∙T)/T_i  e_k+(K_p∙T_d)/T(e_k-2e_(k-1)+e_(k-2))*

For this controller, different parameters are tested, obtaining a satisfactory response with Kp = 50, Kd = 25 and Ki = 0.
Although in this case it is not necessary, as it does not use an integral action, it is good practice in control engineering to use a system that avoids over-actuation due to the accumulation of integral error in the case of having a limited action on the plant. 
For this reason, an anti-windup system is implemented, which restricts the integral action between a maximum and a minimum, preventing the accumulation of an excess of integral error in the event of actuator saturation. It is worth remembering again that this is a solution "applied" in the code but "not used", as the integral gain is zero, acting as a PD regulator.

### 2.3. Machine Learning techniques

#### 2.3.1 Data extraction
To extract data for use in the training of the different machine learning models, one of the previously implemented controllers is used, specifically the PID controller. The 3 measurements described in Table 1 and the performance delivered to each motor, obtained through the PID controller, are collected as data.
The CSV library for Python [5] is used to store the data, which allows information to be saved in a .csv file and which can be imported later. 
For data collection the robot is operated for several laps of a scenario, collecting about 20 000 measurement and performance data.
In order to use this data, infinity and nans must be filtered out beforehand, as certain measurements from the laser scanner could be corrupted and this can disturb the training of the model. These values are filtered out by replacing them with a 0.

#### 2.3.2 Models
For the use of Machine Learning models, the Scikit Learn library is used, which allows a quick and easy implementation.
The following models are used, with default parameters [6][7]:

-	Decision tree
-	Artificial Neural Network 
  
Google Colab has been used to test the Machine Learning models. Once these models were developed and tested, the necessary libraries were installed to make them work in Webots.
To test the models with Google Colab the data was separated into training and test, which is why it is reflected in the code. However, it would be possible to train directly with all the data, performing the performance analysis of the model directly on the simulator, allowing a greater amount of data to be available for training.
The training using the Scikit Learn library is very simple once we already have the data. It is enough to define the model, passing some hyperparameters if it´s needed. Then the model is trained by calling the model.fit() function, passing as a parameter the training data.
 
 ![image](https://user-images.githubusercontent.com/101353583/211430615-343b24d1-a344-4d8f-b883-1751d910af58.png)

Figure 3. Import and train of ML models

The model.predict() function is used to perform the run-time inference of the algorithm, returning the speed setpoints for the wheel motors.

## 3. Scenarios 

In order to test all the developed controllers, two similar but different scenarios are designed. In these scenarios, 20 cm high walls are placed, which is sufficient for the laser scanner to detect them correctly. 
The scenarios are shown in Figure 3 and Figure 4 below:

 ![image](https://user-images.githubusercontent.com/101353583/211430262-dc7b3048-a416-43e5-b008-9d128624c5f3.png)
 
Figure 4. Scenario 1

 ![image](https://user-images.githubusercontent.com/101353583/211430283-e47505d2-8070-4d3b-863d-4c294f20c2be.png)
 
Figure 5. Scenario 2


## 4. Experimental results

The different controllers are tested, obtaining more or less satisfactory results depending on the algorithm used. It is worth mentioning that these results will be analysed in video format, as a complement to this report.

### 4.1 Ruled based controller
This rule-based algorithm is characterised by great simplicity, yet its performance is very satisfactory. 
The robot completes both circuits without problems, some observations are obtained:
	On straight walls it tracks correctly and parallel to the wall.
	When facing walls, it turns correctly and continues tracking.
	There are some oscillations at flush corners, on walls that end abruptly.

### 4.2 PID controller
The performance is very satisfactory, completing both scenarios. Some conclusions are drawn:
	The PID performs constant corrections to keep the robot at a given distance from the wall.
	When a front wall is detected, there is a small overshoot between the actuation of the ruler that rotates the robot and the return to control by the PID.
	The performance of both PIDs is very similar, with a slight improvement for the Tustin discretised PID.


### 4.3 Machine learning techniques
Two techniques are tested with the default parameters, defined in the Scikit Learn documentation [6][7].
As the models have been trained from a PID in the first scenario, the results in this one are not representative because they are biased. The results in the second scenario are more realistic, however the performance is quite similar.
For the decision tree the results are not very good. It manages to track the wall, although there are conflicts when taking curves or with walls that end suddenly. It can get lost and rotate, or go back the way it came in narrow corridors with cut-out walls.
For the neural network, the performance is quite good. As it is trained with data obtained from the PID, the behaviour is quite similar to that obtained with the PID, tracking the walls and completing both maps.

## 5. Final discussion

Several conclusions can be drawn from the exercise, which are set out below. 
For a task that does not require great complexity, such as wall tracking, it was observed that the simplest algorithms are capable of performing it without major problems. In this case, the rule-based algorithm works well enough to be comparable to the use of a PID.
Machine learning models are very susceptible to the data used to train them. In this experiment, the data was obtained using the PID controller, so the behaviour of the machine learning algorithms is a "copy" of the controller, but with noticeably worse performance for the tree and similar for the neural network. In any case, these more complex algorithms have not outperformed the simpler algorithms.
For future work, more tests could be done with more complex scenarios, which could test the rule-based algorithm to see if it is really capable of overcoming these new cases. We could also try to train Machine Learning models in different ways, avoiding that their behaviour is, as in this case, a "copy" of the way of training. We could also train a reinforcement learning model that learns to follow the wall by itself, without needs of previously taken data.

## 6. Bibliographic references

[1] “Webots: robot simulator.” https://cyberbotics.com/ (accessed Jan. 07, 2023).

[2] “Webots documentation: Robotis’ TurtleBot3 Burger.” https://cyberbotics.com/doc/guide/turtlebot3-burger (accessed Jan. 07, 2023).

[3] “Webots documentation: Lidar Sensors.” https://cyberbotics.com/doc/guide/lidar-sensors#robotis-lds-01 (accessed Jan. 07, 2023).

[4] J. J. Salt Llobregat, Á. Cuenca Lacruz, V. Casanova Calvo, and A. Correcher Salvador, Control automático. Tiempo Continuo y Tiempo Discreto. España: Reverté, 2016.

[5] “CSV File Reading and Writing — Python 3.11.1 documentation.” https://docs.python.org/3/library/csv.html (accessed Jan. 07, 2023).

[6] “1.10. Decision Trees — scikit-learn 1.2.0 documentation.” https://scikit-learn.org/stable/modules/tree.html (accessed Jan. 07, 2023).

[7] “sklearn.neural_network.MLPRegressor — scikit-learn 1.2.0 documentation.” https://scikit-learn.org/stable/modules/generated/sklearn.neural_network.MLPRegressor.html (accessed Jan. 07, 2023
