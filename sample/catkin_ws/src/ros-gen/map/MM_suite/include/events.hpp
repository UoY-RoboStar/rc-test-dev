#ifndef _H_EVENTS
#define _H_EVENTS

#include "event.hpp"
#include "eventio.hpp"
#include "inputadapter.hpp"
#include "types.hpp"

class inpIN : public TypedInputEvent<unsigned int,sample::Input> {
  public:
  	// Inherit constructors
  	using TypedInputEvent::TypedInputEvent;
  	
	// Name
  	std::string getName() const override { return "inp"; };
  	
  	// User provided implementations
  	sample::Input getROSMessage() override;
  	bool compare(sample::InputAccepted::ConstPtr message) override;
};
class outpOUT : public TypedOutputEvent<Value::Value> {
  public:
  	// Inherit constructors
  	using TypedOutputEvent::TypedOutputEvent;
  	
  	// Name
  	std::string getName() const override { return "outp"; };
  	
  	// User provided implementations
  	bool compare(sample::Output::ConstPtr message) override;
};

#endif /* _H_EVENTS */
