# README
## Silverback Code Library
### How To Use
#### Installation
To use this library most effectively, download the GitHub application for your computer. Once installed you will have the option to *clone* this repository. Clone it to `Documents\Arduino\libraries`. This will allow for easy updates and usage within Arduino.

Once in your libraries folder you can view the example sketches in Arduino through `File -> Examples -> Silverback -> ...`. The examples show how to effectively use each library.

#### Updating
##### Pull an Update
To update your library open the *Git Shell* application and type: *(without the `$`)*  
`$ cd ../Arduino/libraries/Silverback`  
`$ git pull origin master`  
This will reset your library to the most up-to-date version from the online repository and will be ready to use the next time you open Arduino.
##### Push an update
If you have made changes to the repository, like adding a file, or modifying an existing file, you must make a commit and a pull request. Visit [this link](http://dont-be-afraid-to-commit.readthedocs.io/en/latest/git/commandlinegit.html) to learn how to do this with the *Git Shell* command line. You can also do this through the *GitHub App* on [Windows](https://github.com/blog/1969-create-pull-requests-in-github-for-windows) or on [Mac](https://github.com/blog/1946-create-pull-requests-with-github-for-mac).

### Technical Information
#### **Mega Pin-Table**  

 **Mega Pin** | **Use** | **Description**
---:|:---:| ---
(RX0) 0 |     |  		
(TX0) 1 |     |  
~2      |M1INA|	Motor 1 direction input A
~3      |		  |
~4	    |M1INB|	Motor 1 direction input B
~5	    |     |
~6	    |M1EN/DIAG|	Motor 1 enable input/fault output
~7	    |M2INA|	Motor 2 direction input A
~8	    |M2INB|	Motor 2 direction input B
~9	    |M1PWM|	Motor 1 speed input
~10	    |M2PWM|	Motor 2 speed input
~11	    |XBRX |	Xbee digital receive (modified)
~12	    |M2EN/DIAG|	Motor 2 enable input/fault output
~13	    |XBTX |	Xbee digital transmit (modified)
(TX3) 14|	IN1 |	Motor 3 pos speed input
(RX3) 15|	IN2 |	Motor 3 neg speed input
(TX2) 16|     |		
~       | ~   |~
23      |     |		
24	    |ENC1A|	Motor 1 encoder output A
25	    |ENC2A|	Motor 2 encoder output A
26	    |ENC3A|	Motor 3 encoder output A
27	    |     |
~       |~    |~
~44	    |     |
~45	    |SRV1 |	Servo 1 PWM signal
~46	    |SRV2 |	Servo 2 PWM signal
47	    |     |
~       |~    |~
53	    |     |
A0	    |M1CS |	Motor 1 current sense output
A1	    |M2CS |	Motor 2 current sense output
A2	    |DMSR |	Digital measuring sensor (R)
A3	    |DMSL |	Digital measuring sensor (L)
A4      |HES  | Hall effect sensor  
