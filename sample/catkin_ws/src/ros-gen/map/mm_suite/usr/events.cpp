#include "event.hpp"
#include "events.hpp"
#include "executor.hpp"
#include "traceexecutor.hpp"
#include "types.hpp"

bool outpOUT::compare(sample::Output::ConstPtr message) {
	// To be completed.
	if (message->value == 1) {
		Value::Value val = Value::High{};
		return value == val;
	} else if (message->value == 0) {
		Value::Value val = Value::Low{};
		return value == val;
	} else {
		return false;
	}

};
bool inpIN::compare(sample::InputAccepted::ConstPtr message) {
	// To be completed.
	return true;
};
sample::Input inpIN::getROSMessage() {
	// To be completed.
	sample::Input msg;

	msg.value = value;
	return msg;
};
