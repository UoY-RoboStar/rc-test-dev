#ifndef _H_MESSAGES
#define _H_MESSAGES

#include <optional>
#include "ros/ros.h"
#include "message.hpp"

class outputMessage : public TopicMessage<sample::Output::ConstPtr> {
  public:
  	std::string getName() {
  		return "outputMessage";
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->stamp;
  	};
};

class input_acceptedMessage : public TopicMessage<sample::InputAccepted::ConstPtr> {
  public:
  	std::string getName() {
  		return "input_acceptedMessage";
  	};
  	
  	std::optional<ros::Time> getTime() const override {
  		return message->stamp;
  	};
};

class inputMessage : public TopicMessage<sample::Input::ConstPtr> {
  public:
  	std::string getName() {
  		return "inputMessage";
  	};
};



#endif /* _H_MESSAGES */
