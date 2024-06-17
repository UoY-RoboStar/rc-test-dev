#include "event.hpp"
#include "events.hpp"
#include "executor.hpp"
#include "traceexecutor.hpp"
#include "types.hpp"

bool outpOUT::compare(sample::Output::ConstPtr message) {
	// To be completed.
	if (message->value < 5) {
		return value == 2;
	} else if (message->value >= 5) {
		return value == 5;
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

	if ((Value::Value) Value::Low{} == value) {
		msg.value = 0;
	} else if ((Value::Value) Value::High{} == value) {
		msg.value = 1;
	}
	return msg;
};
