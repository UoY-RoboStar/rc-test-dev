#include <memory>
#include "event.hpp"
#include "messages.hpp"
#include "tester.hpp"
#include "traceexecutor.hpp"

// Callbacks for every topic published by the node under test
void Tester::outputCallback(sample::Output::ConstPtr message) {
	auto omsg = std::make_shared<outputMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for output.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}
// Acknowledgment callbacks
void Tester::input_acceptedCallback(sample::InputAccepted::ConstPtr message) {
	auto omsg = std::make_shared<input_acceptedMessage>();
	omsg->message = message;
	#ifdef DEBUG
		ROS_INFO_STREAM("Callback for input_accepted.");
	#endif
	exec.put(omsg);
	exec.getSMessage(omsg);
}

// Callbacks for instrumentation topics for services that we act as a client.


// Callbacks for instrumentation topics for services that we act as a server.

