#include <algorithm>
#include <cstdlib>
#include <vector>

template <typename InputIteratorA, typename InputIteratorB>
void get_indices(InputIteratorA first, InputIteratorA last, InputIteratorB firstb, bool cond)
{
	while (first++ != last) {
		if (*first == cond) {
			firstb++ = *first;
		}
	}
}

struct Vec3s {
	float x, y, z;
};

struct Velocities {
	Vec3s flip, pic;
};

template <typename T>
std::vector<int> indices_if(const std::vector<T> &input, const bool cond)
{
	size_t size = input.size();
	std::vector<int> indices;

	for (size_t i(0); i < size; ++i) {
		if (input[i] == cond) {
			indices.push_back(i);
		}
	}

	return indices;
}

template <typename T>
void stable_partition_indices(const std::vector<T> &input, const std::vector<int> indices)
{
	for (const auto index : indices) {
		auto it = input.begin() + index;
		std::rotate(it, it + 1, input.end());
	}
}

