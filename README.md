# H2Rover Example Library Installation

Created By: Daniel Benusovich  
Created On: 4 November 2017  
Last Edited By: Daniel Benusovich  
Edited On: 11 November 2017  

## Purpose of Example Library

The purpose of these examples are to provide a simple implementation of the H2rOver library. Rather than relying on only comments, it is often prudent to have an example to follow while coding complex systems. This example library will provide various examples for the implementation of the various sub-systems and allow for faster development times. 
  
## Requirements

	1. Arduino Sketchbook is installed
	   - Install here: https://www.arduino.cc/en/Guide/HomePage 
	     
	2. Git is installed on the computer
	   - Install here: https://git-scm.com/book/en/v2/Getting-Started-Installing-Git 
	   - Verify installation by running the command “git” into the command line.
  
 	 3. The following libraries are installed:
	    1. Adafruit Unified Sensor 
	    2. Adafruit BNO055
	    3. Xbee
	    4. H2Rover

## Cloning the Repository

  	1. Navigate to the directory where Arduino is installed and enter the libraries folder.
      	Example Directory: C:\Program Files (x86)\Arduino\examples
	
  	2. Open a terminal inside the libraries folder.
      	Execute this command:
        	a. git clone https://github.com/H2rOver/H2RoverExamples.git 
		
A folder should appear in the libraries folder named H2RoverExamples
  
## Verification of Repository Import

  	1. In Arduino Sketchbook navigate to the Examples Tab
      	Check under the Examples tab for H2RoverExamples
If it is there you are done! Click on the library in the tab to import it into your project.
  
If the library is not there please delete the H2RoverExamples folder from its current location and try again. The repository has been cloned into the wrong folder so please start again from Cloning the Repository and find the proper location of the Arduino installation folder. 
  
## Updating the Library

  The library will be periodically updated requiring one to run a command to update the library on local machines. Navigate to the 
  folder where the library is contained and run the following command to update the repository:
  
  	git pull


