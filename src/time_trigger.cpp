#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <flight_control/jetsonTX2GPIO.h>
#include <sensor_msgs/Image.h>

using namespace std;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}
sensor_msgs::Image cam;
void Callback(const sensor_msgs::Image::ConstPtr& msg)
{
  cam = *msg;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "time_trigger");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("/camera/image_raw", 1000, Callback);

    cout << "Testing the GPIO Pins" << endl;
    // was originally gpio219 for the TX1
    TX2_GPIO redLED = gpio298 ;    // Ouput
    // was originally gpio38 for the TX1
    TX2_GPIO pushButton = gpio481 ; // Input
    // Make the button and led available in user space
   
    gpio_export(pushButton) ;
    gpio_export(redLED) ;
    gpio_set_direction(pushButton,GPIO_DIRECTION_INPUT) ;
    gpio_set_direction(redLED,GPIO_DIRECTION_OUTPUT) ;
    GPIO_PIN_VALUE value = GPIO_PIN_VALUE_LOW;
    gpio_set_value(redLED,GPIO_PIN_VALUE_LOW);
    ros::Time trigger = ros::Time::now();
    ros::Time past;
    while (ros::ok()&&getkey() != 27)
    {

    if((ros::Time::now() - trigger) > ros::Duration(0.01)){
        gpio_get_value(redLED, &value) ;
        if(value==GPIO_PIN_VALUE_HIGH){
        //cout << "H->L" << endl;
        gpio_set_value(redLED, GPIO_PIN_VALUE_LOW);
        //cout << "t=" <<ros::Time::now()-past<< endl;
        past=ros::Time::now();
        }
        else{
        //cout << "L->H" << endl;
        gpio_set_value(redLED, GPIO_PIN_VALUE_HIGH);
        }
        //cout << "dt=" <<cam.header.stamp-trigger<< endl;
        //cout << "t=" <<ros::Time::now() - trigger<< endl;

        //usleep(5000);         //5ms,100hz,35fps
        trigger = ros::Time::now();
        }
    else{}
    cout << "trigger:"<<past<< endl;
    //cout << "camera: "<<cam.header.stamp<< endl;

    ros::spinOnce();
    }

    gpio_unexport(redLED);     // unexport the LED
    gpio_unexport(pushButton);      // unexport the push button

    return 0;
}



