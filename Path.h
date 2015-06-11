#include <vector>
#include <tuple>
#include "Error.h"

struct point
{
	double x;
	double y;
	double z;

	inline point operator+(const point& other) const {
		point res{ x + other.x, y + other.y, z + other.z };
		return res;
	}

	inline point operator-(const point& other) const {
		point res{ x - other.x, y - other.y, z - other.z };
		return res;
	}


	inline point operator*(const point& other) const { //dot operator
		point res{ x * other.x, y * other.y, z * other.z };
		return res;
	}


	inline bool operator == (const point& other) const {
		
		return x == other.x && y == other.y && z == other.z;
	}

};
struct velocity
{
	double x;
	double y;
	double z;
};
struct acceleration
{
	double x;
	double y;
	double z;
};
typedef unsigned long int timestamp;
namespace Shapes
{

	class Path
	{
	public:

		~Path();
		//Constructors
		Path();
		Path(size_t size);
		Path(size_t size, timestamp period);
		Path(char* filename);
		Path(std::vector<point> given_points, timestamp period);
		Path(std::vector<point> given_points, std::vector<timestamp> variable_periods);


		Error dump_to_file(char* filename);
		Error read_points_from_file(char* filename);

		Error read_accel_from_file(char* filename);

		point get_point(timestamp time);

		Error add_point(point added_point); //only for fixed_period
		Error add_point(point added_point, timestamp time); //if fixed_period, will make dynamic timing

		std::vector<point> get_points_data();
		std::vector<timestamp> get_time_data();
		timestamp get_fixed_period_data();

		Path combine_paths(Path other_path, velocity initial_velocity, velocity their_velocity);
	private:

		void calculate_velocities_from_position(velocity initial_velocity);
		unsigned int get_index_from_time(timestamp time); // O(log n)

		timestamp fixed_period;
		void convert_fixed_period_to_timestamps();

		//A map would also make sense, since
		std::vector<point> points;

		std::vector<timestamp> timestamps; //must be ordered

		
		std::vector<velocity> velocities;
		bool vecolities_calculated; //we might lazily calculate them, or need to recalculate them later on if we add points


		timestamp get_time(unsigned int index); 
		point get_position_at_time(timestamp time);
		velocity static get_velocity_from_finite_difference(point a, point b, timestamp dt);

	};

}
