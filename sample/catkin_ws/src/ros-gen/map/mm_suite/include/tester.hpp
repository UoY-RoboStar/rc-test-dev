#ifndef _H_TESTER
#define _H_TESTER

#include <memory>
#include "ros/ros.h"
#include "events.hpp"
#include "verdict.hpp"
#include "traceexecutor.hpp"

class Tester {
  private:
    std::vector<std::shared_ptr<Message>> events;
	Verdict verdict = Verdict::Inconclusive;
	TraceExecutor exec;
	
	ros::NodeHandle nh;
	
  public:
  
	// NUT Publishers that we subscribe
	ros::Subscriber output_sub = nh.subscribe("/sample/output", 10, &Tester::outputCallback, this);
	
	// NUT Instrumentation that we subscribe
	ros::Subscriber input_accepted_sub = nh.subscribe("/sample/input_accepted", 10, &Tester::input_acceptedCallback, this);
	
	// NUT Subscribers that we publish on
	ros::Publisher input_pub = nh.advertise<sample::Input>("/sample/input", 1000);
	
	// Services that the NUT provides and that we therefore use as a client
	
	// Serivces that the NUT calls and that we therefore provide as a server
	
	void ready() {
		bool ready = false;

		while (!ready) {
			ready = output_sub.getNumPublishers() > 0
					&&input_accepted_sub.getNumPublishers() > 0
					;
		}
	};
	
	ros::Timer regular_timer = nh.createTimer(ros::Duration(0.5), &Tester::_periodicTimeCallback, this, false, true);
	ros::Timer final_timer;
	
  	// Timer callbacks
  	void _finalTimeoutCallback(const ros::TimerEvent& event) {
  		ROS_INFO_STREAM("Final timeout reached.");
  		exec.shutdown();
  	};
  	void _periodicTimeCallback(const ros::TimerEvent& event) {
  		exec.update();
  	};
  	
    // Callbacks for every topic published by the node under test
    void outputCallback(sample::Output::ConstPtr message);
    
    // Acknowledgment callbacks
    void input_acceptedCallback(sample::InputAccepted::ConstPtr message);
   	
    // Service callbacks for every service called by the node under test
    
    // Setup events
    void setup(std::shared_ptr<Event> event) {
    	if (auto ev = std::dynamic_pointer_cast<inpIN>(event)) {
    		auto adapter = std::make_shared<TopicInputAdapter<sample::Input>>(&input_pub);
    		ev->setAdapter(adapter);
    	}
    };
    

    // Standard methods
    Verdict getVerdict() { return verdict; };
    
    void setTrace(std::vector<std::shared_ptr<Event>> ex) {
		exec.setTrace(ex);
		final_timer = nh.createTimer(ros::Duration(1+0.5*ex.size())+ros::Duration(1), &Tester::_finalTimeoutCallback, this, true, true);
	};
	std::vector<std::shared_ptr<Message>> getMessages() {
		return exec.getMessages();
	};
	TraceExecutor* getExecutor() {
		return &exec;
	};
};

#endif
