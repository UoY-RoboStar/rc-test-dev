#ifndef _H_MESSAGE
#define _H_MESSAGE

#include <memory>
#include <optional>
#include <string>
#include "event.hpp"
#include "ros/ros.h"

class Message {
  protected:
	std::optional<ros::Time> time = std::nullopt;
  public:
	virtual std::optional<ros::Time> getTime() const { return time; };
	virtual void setTime(ros::Time t) { time = t; };

	virtual std::string getName() {
	  return "Unknown";
	};

	virtual std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
	#ifdef DEBUG
	  ROS_INFO_STREAM("No match for Message");
	#endif
	  return std::nullopt;
	};

	/*
	 * Compares the supplied Event with this message, returning true
	 * if they match. This method is used to check a trace of events
	 * against a trace of messages.
	 */
	virtual bool compare(const Event& ev) const { return false; };
	virtual void respond(const Event& ev) const { };

	virtual bool compare(std::shared_ptr<Event> ev) const { return false; };
	virtual std::string getFullName() {
		if (getTime().has_value()) {
			return getName() + " (" + std::to_string(getTime().value().sec) + "." + std::to_string(getTime().value().nsec) + ")";
		} else {
			return getName() + " ()";
		}
	};
};

bool LTHAN(std::shared_ptr<Message> m1, std::shared_ptr<Message> m2);

// Template for ROS Topic Messages
template <typename M>
class TopicMessage : public Message {
  public:
	M message;

	std::string getName() {
	  return "Unknown TopicMessage";
	};
//	bool compare(const Event& ev) const {
//	  return ev.compare(message);
//	}
	bool compare(std::shared_ptr<Event> ev) const {
	  return ev->compare(message);
	}
	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
	  if (std::dynamic_pointer_cast<TopicMessage<M>>(m)) {
		#ifdef DEBUG
		ROS_INFO_STREAM("Match successful for TopicMessage<M>");
		#endif
		return m;
	  } else {
		return std::nullopt;
	  }
	};
};

template <typename M>
class ServiceMessage : public Message {
  public:
	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) {
	  if (std::dynamic_pointer_cast<ServiceMessage<M>>(m)) {
		#ifdef DEBUG
		ROS_INFO_STREAM("Match successful for ServiceMessage<M>");
		#endif
		return m;
	  } else {
		return std::nullopt;
	  }
	};
};

template <typename M>
class ServiceResponseMessage : public ServiceMessage<M> {
  private:
    M* response;
  public:
	ServiceResponseMessage(std::optional<ros::Time> _time, M* _response) {
		time = _time;
	}
	ServiceResponseMessage(M* _response) {
		response = _response;
	}
	M* getResponse() {
		return response;
	}
	bool compare(std::shared_ptr<Event> ev) override {
		#ifdef DEBUG
			ROS_INFO("Hit compare of %s.", this->getName());
		#endif
		return ev->compare(response);
	}
	std::optional<std::shared_ptr<Message>> match(std::shared_ptr<Message> m) override {
      ROS_INFO("Trying to match %s.", this->getName());
      if (auto mcast = std::dynamic_pointer_cast<ServiceResponseMessage<M>>(m)) {
        ROS_INFO("Match successful for %s.", this->getName());
        return std::make_shared<ServiceResponseMessage<M>>(time,mcast->getResponse());
      } else {
        return std::nullopt;
      }
    };
};

#endif /* _H_MESSAGE */
