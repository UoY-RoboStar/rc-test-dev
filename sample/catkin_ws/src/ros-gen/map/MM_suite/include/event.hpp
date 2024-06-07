#ifndef _H_EVENT
#define _H_EVENT

#include "ros/ros.h"

// Services

// Topics
#include "sample/Output.h"
#include "sample/InputAccepted.h"
#include "sample/Input.h"

class Event {
	public:
		virtual ~Event() = default;
		virtual std::string getName() const { return "Unknown"; };
		
		virtual bool compare(sample::Output::ConstPtr message) { return false; };
		virtual bool compare(sample::InputAccepted::ConstPtr message) { return false; };
		virtual bool compare(sample::Input::ConstPtr message) { return false; };


		virtual void publish() { 
		#ifdef DEBUG
			ROS_INFO_STREAM("publish called on base class of Event."); 
		#endif
		};
		virtual bool isSUTInput() const { return false; };
};

#endif /* _H_EVENT */
