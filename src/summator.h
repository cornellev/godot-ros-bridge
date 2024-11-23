/* summator.h */

#ifndef SUMMATOR_H
#define SUMMATOR_H

#include <godot_cpp/classes/ref.hpp>


namespace godot {
class Summator : public RefCounted {
	GDCLASS(Summator, RefCounted);

	int count;

protected:
	static void _bind_methods();

public:
	void add(int p_value);
	void reset();
	int get_total() const;

	Summator();
};
}

#endif // SUMMATOR_H