#ifndef _H_TYPE_DECLS
#define _H_TYPE_DECLS

#include <tuple>
#include <variant>

namespace Value {
	
	class High {
	  public:
		
		friend bool operator ==(const High& lhs, const High& rhs) {
			return std::tie() == std::tie();
		}
	};
	class Low {
	  public:
		
		friend bool operator ==(const Low& lhs, const Low& rhs) {
			return std::tie() == std::tie();
		}
	};
	
	typedef std::variant<Value::High,Value::Low> Value;
}

#endif /* _H_TYPE_DECLS */
