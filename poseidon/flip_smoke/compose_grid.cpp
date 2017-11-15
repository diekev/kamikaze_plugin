#include <algorithm>

/* Input Types */
enum {
	FLUID_SOURCE,
	FLUID_SINK,
	FLUID_COLLISION,
};

/* Field Types */
enum {
	FIELD_HEAT,
	FIELD_DENSITY,
	FIELD_FLAME,
	FIELD_PRESSURE,
	FIELD_REACTION,
};

/* Blend Types */
enum {
	GRID_BLEND_OVER     = 0,
	GRID_BLEND_ADD      = 1,
	GRID_BLEND_SUBRACT  = 2,
	GRID_BLEND_MIN      = 3,
	GRID_BLEND_MAX      = 4,
	GRID_BLEND_MULTIPLY = 5,
};

struct FluidDescr {
	float value;  // e.g: density value
	float vel[3]; // velocity, if collision object
	float alpha;  // alpha mask used for blending
	short type;   // input type
	short field;  // field to influence
	short blend;  // blend type
};

float lerp(float u, float v, float a)
{
	return (1.0f - a) * u + a * v;
}

void compose_grids(float *field, float *input_field, float alpha, int type)
{
	int index = 0;
	float u = field[index];
	float v = input_field[index];

	switch (type) {
		case GRID_BLEND_OVER:
			field[index] = lerp(u, v, alpha);
			break;
		case GRID_BLEND_ADD:
			field[index] = lerp(u, u + v, alpha);
			break;
		case GRID_BLEND_SUBRACT:
			field[index] = lerp(u, u - v, alpha);
			break;
		case GRID_BLEND_MIN:
			field[index] = lerp(u, std::min(u, v), alpha);
			break;
		case GRID_BLEND_MAX:
			field[index] = lerp(u, std::max(u, v), alpha);
			break;
		case GRID_BLEND_MULTIPLY:
			field[index] = lerp(u, u * v, alpha);
			break;
	}
}

